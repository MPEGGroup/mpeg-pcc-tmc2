cmake_minimum_required(VERSION 3.17 FATAL_ERROR)

include(FetchContent)
set( FETCHCONTENT_QUIET off )

set( lapack_SOURCE_DIR  ${CMAKE_SOURCE_DIR}/dependencies/lapack )
message("Fetch LAPACK libraries: ${lapack_SOURCE_DIR}") 
FetchContent_Declare( lapack
    GIT_REPOSITORY https://github.com/Reference-LAPACK/lapack.git 
    GIT_TAG        v3.12.0
    SOURCE_DIR     ${lapack_SOURCE_DIR}
    GIT_PROGRESS   TRUE )              

# Set some LAPACK options if needed before make available
set(CBLAS ON CACHE BOOL "Build CBLAS" FORCE)
set(LAPACKE ON CACHE BOOL "Build LAPACKE" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build testing" FORCE)

# MAKE AVAILABLE
FetchContent_MakeAvailable( lapack )
set(LAPACK_LIBRARIES lapack blas)
