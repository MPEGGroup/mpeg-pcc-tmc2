CMAKE_MINIMUM_REQUIRED (VERSION 2.8.11)

GET_FILENAME_COMPONENT(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
STRING(REPLACE " " "_" MYNAME ${MYNAME})
PROJECT(${MYNAME} C CXX)

FILE(GLOB SRC  include/*.h 
               source/*.cpp  
               ${HM_LIB_SOURCE_DIR}/TAppCommon/* )

INCLUDE_DIRECTORIES( include 
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include/
                     ${HM_LIB_SOURCE_DIR}/TAppCommon/
                     ${HM_LIB_SOURCE_DIR}/ )

ADD_LIBRARY( ${MYNAME} ${LINKER} ${SRC} )

TARGET_LINK_LIBRARIES( ${MYNAME} PccLibCommon TLibCommon TLibVideoIO TLibEncoder  )

SET_TARGET_PROPERTIES( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)
