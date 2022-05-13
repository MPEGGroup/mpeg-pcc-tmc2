cmake_minimum_required(VERSION 3.17 FATAL_ERROR)

include(FetchContent)
set( FETCHCONTENT_QUIET off )

# VVENC
set( vvenc_SOURCE_DIR  ${CMAKE_SOURCE_DIR}/dependencies/vvenc )
message("Fetch VVENC libraries: ${vvenc_SOURCE_DIR}") 
FetchContent_Declare( VVENC GIT_REPOSITORY https://github.com/fraunhoferhhi/vvenc.git 
                            GIT_TAG        v1.4.0
                            SOURCE_DIR     ${vvenc_SOURCE_DIR}
                            GIT_PROGRESS   TRUE )              
set(vvenc_ADD_SUBDIRECTORIES "source/Lib/apputils;source/App/vvencFFapp")

# VVDEC
set( vvdec_SOURCE_DIR ${CMAKE_SOURCE_DIR}/dependencies/vvdec )
message("Fetch VVDEC libraries: ${vvdec_SOURCE_DIR}") 
FetchContent_Declare( VVDEC GIT_REPOSITORY https://github.com/fraunhoferhhi/vvdec.git
                            GIT_TAG        v1.5.0
                            SOURCE_DIR     ${vvdec_SOURCE_DIR}
                            GIT_PROGRESS   TRUE )
set(vvdec_ADD_SUBDIRECTORIES "source/App/vvdecapp")

# MAKE AVAILABLE
fetchcontent_makeavailable( VVENC VVDEC )
if (NOT MSVC)
  target_compile_options(vvenc PUBLIC "-w")   
  target_compile_options(vvdec PUBLIC "-w")        
endif()
set_property(TARGET vvenc apputils vvencFFapp PROPERTY FOLDER "VVenC")
set_property(TARGET vvdec vvdecapp            PROPERTY FOLDER "VVdeC")
target_compile_features( vvenc PUBLIC cxx_std_14 )
target_compile_features( vvdec PUBLIC cxx_std_14 )
