cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

SET( JM_VERSION         jm19.0_app )
SET( JM_DIR             ${CMAKE_SOURCE_DIR}/dependencies/${JM_VERSION}/ )
SET( JM_APP_SOURCE_DIR  ${JM_DIR}/source )
MESSAGE("Clone JM application: ${JM_DIR}") 
  
IF( NOT EXISTS "${JM_DIR}/CMakeLists.txt" )   
  MESSAGE("  - JM application clone")
  EXECUTE_PROCESS( COMMAND git -c http.sslVerify=false clone --depth 1 https://vcgit.hhi.fraunhofer.de/jct-vc/JM.git ${JM_DIR} )
  #EXECUTE_PROCESS( COMMAND git apply ${CMAKE_SOURCE_DIR}/dependencies/jm-modification/SampleStreamITRI.patch WORKING_DIRECTORY ${JM_DIR} )    
ENDIF()
