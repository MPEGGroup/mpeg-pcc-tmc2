cmake_minimum_required(VERSION 3.17 FATAL_ERROR)

  
SET( HDRTOOLS_DIR             ${CMAKE_SOURCE_DIR}/dependencies/HDRTools )
MESSAGE("Clone HDRTools libraries: ${HDRTOOLS_DIR}") 

IF( NOT EXISTS "${HDRTOOLS_DIR}/CMakeLists.txt" )   
  MESSAGE("  - HDRTools clone")
  EXECUTE_PROCESS( COMMAND git clone --depth 1 http://gitlab.com/standards/HDRTools.git ${HDRTOOLS_DIR} )  
ENDIF()

IF ( NOT EXISTS "${HDRTOOLS_DIR}/common/CMakeLists.txt" )
  message( "\n\n Error: commande: \n \t  git clone http://gitlab.com/standards/HDRTools.git ${HDRTOOLS_DIR} not work")
  message( FATAL_ERROR "${HDRTOOLS_DIR}/common/CMakeLists.txt not exist")
ELSE()
  ADD_SUBDIRECTORY( ${HDRTOOLS_DIR}/common ) 
ENDIF()
