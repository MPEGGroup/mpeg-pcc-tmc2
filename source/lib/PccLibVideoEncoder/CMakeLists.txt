CMAKE_MINIMUM_REQUIRED (VERSION 2.8.11)

GET_FILENAME_COMPONENT(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
STRING(REPLACE " " "_" MYNAME ${MYNAME})
PROJECT(${MYNAME} C CXX)

FILE(GLOB SRC include/*.h 
              source/*.cpp
              ${HM_LIB_SOURCE_DIR}/TAppCommon/* )

SET( LIBS PccLibCommon )
INCLUDE_DIRECTORIES( include 
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include/
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include/
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibVideoEncoder/include/ )
IF( USE_HMLIB_VIDEO_CODEC )
  INCLUDE_DIRECTORIES( ${HM_LIB_SOURCE_DIR}/ )
  SET( LIBS ${LIBS} TLibCommon TLibEncoder )
ENDIF()
IF( USE_FFMPEG_VIDEO_CODEC )
  INCLUDE_DIRECTORIES( ${FFMPEG_INCLUDE} )
  INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/source/lib/PccLibVideoDecoder/include/ )
  SET( LIBS ${LIBS} ${AVCODEC_LIBRARY} ${AVUTIL_LIBRARY} ${AVFORMAT_LIBRARY} PccLibVideoDecoder )    
ENDIF()

ADD_LIBRARY( ${MYNAME} ${LINKER} ${SRC} )

TARGET_LINK_LIBRARIES( ${MYNAME} ${LIBS} )

SET_TARGET_PROPERTIES( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)
