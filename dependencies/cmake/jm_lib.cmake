cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

SET( JM_VERSION         jm19.0_lib )
SET( JM_DIR             ${CMAKE_SOURCE_DIR}/dependencies/${JM_VERSION}/ )
SET( JM_LIB_SOURCE_DIR  ${JM_DIR}/source )
MESSAGE("Clone JM libraries: ${JM_DIR}") 
  
IF( NOT EXISTS "${JM_DIR}/CMakeLists.txt" )   
  MESSAGE("  - JM library clone")
  EXECUTE_PROCESS( COMMAND git -c http.sslVerify=false clone --depth 1 https://vcgit.hhi.fraunhofer.de/jct-vc/JM.git ${JM_DIR} ) 
  EXECUTE_PROCESS( COMMAND git apply ${CMAKE_SOURCE_DIR}/dependencies/jm-modification/PCC_JM.patch WORKING_DIRECTORY ${JM_DIR} )  
ENDIF()

set( CMAKE_C_STANDARD 11 )
set( CMAKE_C_STANDARD_REQUIRED ON )

file(GLOB JMCOMFILES "${JM_LIB_SOURCE_DIR}/lib/lcommon/*.h" "${JM_LIB_SOURCE_DIR}/lib/lcommon/*.c")
file(GLOB JMENCFILES "${JM_LIB_SOURCE_DIR}/app/lencod/*.h" "${JM_LIB_SOURCE_DIR}/app/lencod/*.c")
file(GLOB JMDECFILES "${JM_LIB_SOURCE_DIR}/app/ldecod/*.h" "${JM_LIB_SOURCE_DIR}/app/ldecod/*.c")

add_library( lencod ${JMCOMFILES} ${JMENCFILES} )
SET_TARGET_PROPERTIES( lencod PROPERTIES LINKER_LANGUAGE C)
target_compile_features(lencod PUBLIC cxx_std_11)
target_include_directories(lencod PUBLIC ${JM_LIB_SOURCE_DIR}/lib/lcommon ${JM_LIB_SOURCE_DIR}/app/lencod)

add_library( ldecod ${JMCOMFILES} ${JMDECFILES} )
SET_TARGET_PROPERTIES( ldecod PROPERTIES LINKER_LANGUAGE C)
target_compile_features(ldecod PUBLIC cxx_std_11)
target_include_directories(ldecod PUBLIC ${JM_LIB_SOURCE_DIR}/lib/lcommon ${JM_LIB_SOURCE_DIR}/app/ldecod)
