CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

GET_FILENAME_COMPONENT(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
STRING(REPLACE " " "_" MYNAME ${MYNAME})
SET( MYNAME ${MYNAME}${CMAKE_DEBUG_POSTFIX} )
PROJECT(${MYNAME} C CXX)

FILE(GLOB SRC *.h *.cpp *.c ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite/* )

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include                    
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibDecoder/include                
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibMetrics/include
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibConformance/include
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include 
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamReader/include
                     ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite
                     ${CMAKE_SOURCE_DIR}/dependencies/tbb/include 
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibConformance/include )

ADD_EXECUTABLE( ${MYNAME} ${SRC} )

SET( LIBS PccLibCommon PccLibDecoder PccLibMetrics PccLibConformance tbb_static PccLibBitstreamCommon PccLibBitstreamReader )

IF( USE_FFMPEG_VIDEO_CODEC )
  IF( WIN32 OR MSVC OR MSYS OR MINGW )
    LINK_DIRECTORIES( ${FFMPEG_DIR}/lib ) 
  ENDIF()  
ENDIF()

TARGET_LINK_LIBRARIES( ${MYNAME} ${LIBS} )

INSTALL( TARGETS ${MYNAME} DESTINATION bin )
