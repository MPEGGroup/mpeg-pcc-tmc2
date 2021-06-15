CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

GET_FILENAME_COMPONENT(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
STRING(REPLACE " " "_" MYNAME ${MYNAME})
PROJECT(${MYNAME} C CXX)

ENABLE_LANGUAGE( C )

INCLUDE(CheckSymbolExists)
CHECK_SYMBOL_EXISTS( getrusage sys/resource.h HAVE_GETRUSAGE )

CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/include/PCCConfig.h.in
                ${CMAKE_CURRENT_SOURCE_DIR}/include/PCCConfig.h )

FILE(GLOB PROJECT_INC_FILES include/*.h
                            ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.hpp
                            ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.h
                            ${CMAKE_SOURCE_DIR}/dependencies/libmd5/*.h )
FILE(GLOB PROJECT_CPP_FILES source/*.cpp 
                            ${CMAKE_SOURCE_DIR}/dependencies/libmd5/*.c )
FILE(GLOB PROJECT_IN_FILES  *.in )

SOURCE_GROUP( inc   FILES ${PROJECT_INC_FILES} )
SOURCE_GROUP( cpp   FILES ${PROJECT_CPP_FILES} )
SOURCE_GROUP( input FILES ${PROJECT_IN_FILES}  )

INCLUDE_DIRECTORIES( include
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include 
                     ${CMAKE_SOURCE_DIR}/dependencies/nanoflann
                     ${CMAKE_SOURCE_DIR}/dependencies/tbb/include
                     ${CMAKE_SOURCE_DIR}/dependencies/libmd5 )
 
ADD_LIBRARY( ${MYNAME} ${LINKER} ${SRC} ${PROJECT_INC_FILES} ${PROJECT_CPP_FILES} ${PROJECT_IN_FILES} )
IF( ENABLE_PAPI_PROFILING )
  TARGET_LINK_LIBRARIES(${MYNAME} ${CMAKE_SOURCE_DIR}/dependencies/papi/src/libpapi.a  )
ENDIF()

TARGET_LINK_LIBRARIES(${MYNAME} PccLibBitstreamCommon )

SET_TARGET_PROPERTIES( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)

