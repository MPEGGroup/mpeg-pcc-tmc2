CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

GET_FILENAME_COMPONENT(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
STRING(REPLACE " " "_" MYNAME ${MYNAME})
SET( MYNAME ${MYNAME}${CMAKE_DEBUG_POSTFIX} )
PROJECT(${MYNAME} C CXX)

FILE(GLOB SRC *.h *.cpp *.c ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite/* 
                            ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.hpp
                            ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.h )

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include 
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibEncoder/include 
                     ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite
                     ${CMAKE_SOURCE_DIR}/dependencies/tbb/include
                     ${CMAKE_SOURCE_DIR}/dependencies/nanoflann  )
                     
ADD_EXECUTABLE( ${MYNAME} ${SRC} )

SET( LIBS PccLibCommon PccLibEncoder tbb_static ) 

TARGET_LINK_LIBRARIES( ${MYNAME} ${LIBS} )

INSTALL( TARGETS ${MYNAME} DESTINATION bin )
