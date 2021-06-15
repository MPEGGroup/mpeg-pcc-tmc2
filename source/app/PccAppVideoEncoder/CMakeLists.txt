CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

GET_FILENAME_COMPONENT(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
STRING(REPLACE " " "_" MYNAME ${MYNAME})
SET( MYNAME ${MYNAME}${CMAKE_DEBUG_POSTFIX} )
PROJECT(${MYNAME} C CXX)

FILE(GLOB SRC *.h *.cpp *.c ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite/* )

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include                    
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include      
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibVideoEncoder/include 
                     ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite  )
                     
ADD_EXECUTABLE( ${MYNAME} ${SRC} )

SET( LIBS PccLibCommon PccLibBitstreamCommon PccLibVideoEncoder ) 

TARGET_LINK_LIBRARIES( ${MYNAME} ${LIBS} )
IF (MSVC)
    SET_TARGET_PROPERTIES( ${MYNAME} PROPERTIES LINK_FLAGS /STACK:4194304 )
ENDIF()

INSTALL( TARGETS ${MYNAME} DESTINATION bin )
