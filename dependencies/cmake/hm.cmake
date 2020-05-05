cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

MESSAGE("Clone and build HM libraries") 
SET( HM_VERSION         HM-16.20+SCM-8.8 )
SET( HM_DIR             ${CMAKE_SOURCE_DIR}/dependencies/${HM}/ )
set( HM_LIB_SOURCE_DIR  ${HM_DIR}/source/Lib )
message( " HM_LIB_SOURCE_DIR = ${HM_LIB_SOURCE_DIR}" )

IF( NOT EXISTS "${CMAKE_SOURCE_DIR}/dependencies/${HM}/README" )   
  EXECUTE_PROCESS( COMMAND svn checkout https://hevc.hhi.fraunhofer.de/svn/svn_HEVCSoftware/tags/${HM_VERSION}/ ${HM_DIR} )
  EXECUTE_PROCESS( COMMAND svn patch ${CMAKE_SOURCE_DIR}/dependencies/hm-modification/pcc_me-ext_for_HM-16.20+SCM-8.8.patch WORKING_DIRECTORY ${HM_DIR}  ) 
ENDIF()

function(add_hm_library module)
  file(GLOB cppSourceFiles "${HM_LIB_SOURCE_DIR}/${module}/*.cpp")
  file(GLOB cSourceFiles "${HM_LIB_SOURCE_DIR}/${module}/*.c")
  file(GLOB headerFiles "${HM_LIB_SOURCE_DIR}/${module}/*.h")
  add_library(${module} ${cppSourceFiles} ${cSourceFiles} ${headerFiles})
  set_property(TARGET ${module} PROPERTY CXX_CLANG_TIDY) # no clang-tidy
  add_library(TMIV::${module} ALIAS ${module})
endfunction()

add_hm_library(libmd5)
target_compile_features(libmd5 PUBLIC cxx_std_11)
target_include_directories(libmd5 PUBLIC "$<BUILD_INTERFACE:${HM_LIB_SOURCE_DIR}>")
   
add_hm_library(TLibCommon)
target_link_libraries(TLibCommon PRIVATE libmd5)
target_compile_features(TLibCommon PUBLIC cxx_std_11)
target_include_directories(TLibCommon PUBLIC "$<BUILD_INTERFACE:${HM_LIB_SOURCE_DIR}>")
target_compile_definitions(TLibCommon PUBLIC "$<$<CXX_COMPILER_ID:MSVC>:_CRT_SECURE_NO_WARNINGS>")
target_compile_options(TLibCommon PUBLIC "$<$<CXX_COMPILER_ID:Clang>:-w>")
target_compile_options(TLibCommon PUBLIC "$<$<CXX_COMPILER_ID:GNU>:-w>")
  
add_hm_library(TLibVideoIO)
target_link_libraries(TLibVideoIO PUBLIC TLibCommon)

add_hm_library(TLibDecoder)
target_link_libraries(TLibDecoder PUBLIC TLibCommon)
 
add_hm_library(TLibEncoder)
target_link_libraries(TLibEncoder PUBLIC TLibCommon TLibVideoIO )
