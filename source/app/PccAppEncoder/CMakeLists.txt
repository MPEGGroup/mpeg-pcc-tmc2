CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

GET_FILENAME_COMPONENT(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
STRING(REPLACE " " "_" MYNAME ${MYNAME})
SET( MYNAME ${MYNAME}${CMAKE_DEBUG_POSTFIX} )
PROJECT(${MYNAME} C CXX)

FILE(GLOB SRC *.h *.cpp *.c ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite/* 
                            ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.hpp
                            ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.h )

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include                    
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibEncoder/include 
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibMetrics/include 
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include 
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamWriter/include
                     ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite
                     ${CMAKE_SOURCE_DIR}/dependencies/tbb/include
                     ${CMAKE_SOURCE_DIR}/dependencies/nanoflann  )
                     
ADD_EXECUTABLE( ${MYNAME} ${SRC} )

SET( LIBS PccLibCommon PccLibEncoder PccLibMetrics tbb_static PccLibBitstreamCommon PccLibBitstreamWriter ) 

IF( USE_FFMPEG_VIDEO_CODEC )
  IF( WIN32 OR MSVC OR MSYS OR MINGW )
    LINK_DIRECTORIES( ${FFMPEG_DIR}/lib ) 
  ENDIF()  
ENDIF()

TARGET_LINK_LIBRARIES( ${MYNAME} ${LIBS} )
IF (MSVC)
    SET_TARGET_PROPERTIES( ${MYNAME} PROPERTIES LINK_FLAGS /STACK:4194304 )
ENDIF()
INSTALL( TARGETS ${MYNAME} DESTINATION bin )
