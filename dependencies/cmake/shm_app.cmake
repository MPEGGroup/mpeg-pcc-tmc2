cmake_minimum_required(VERSION 3.17 FATAL_ERROR)

SET( SHM_VERSION         SHM-12.4 )
SET( SHM_DIR             ${CMAKE_SOURCE_DIR}/dependencies/${SHM_VERSION}/ )
set( SHM_APP_SOURCE_DIR  ${SHM_DIR}/source )
MESSAGE("Clone SHM application: ${SHM_DIR}") 

IF( NOT EXISTS "${SHM_DIR}/COPYING" )
  MESSAGE("  - SHM application clone")
  EXECUTE_PROCESS( COMMAND git clone --depth 1 https://vcgit.hhi.fraunhofer.de/jvet/SHM.git --branch ${SHM_VERSION} ${SHM_DIR} )
ENDIF()
