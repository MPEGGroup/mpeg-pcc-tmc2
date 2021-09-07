cmake_minimum_required(VERSION 3.17 FATAL_ERROR)

PROJECT (TMC2 CXX C )
SET( TMC2_VERSION_MAJOR 15 )
SET( TMC2_VERSION_MINOR 0  )

# Video encoder and decoder library and application:
OPTION( USE_JMAPP_VIDEO_CODEC     "Use JM application to encode and decode video"           TRUE  )  # JMAPP  = 0
OPTION( USE_HMAPP_VIDEO_CODEC     "Use HM application to encode and decode video"           TRUE  )  # HMAPP  = 1
OPTION( USE_SHMAPP_VIDEO_CODEC    "Use SHM application to encode and decode video"          TRUE  )  # SHMAPP = 2    
OPTION( USE_JMLIB_VIDEO_CODEC     "Clone, build and use JM video encoder and decoder"       TRUE  )  # JMLIB  = 3
OPTION( USE_HMLIB_VIDEO_CODEC     "Clone, build and use HM video encoder and decoder"       TRUE  )  # HMLIB  = 4
OPTION( USE_VTMLIB_VIDEO_CODEC    "Clone, build and use VTM video encoder and decoder"      TRUE )   # VTMLIB = 5
OPTION( USE_FFMPEG_VIDEO_CODEC    "Clone, build and use FFMPEG video encoder and decoder"   FALSE )  # FFMPEG = 6                                                                                                 #UNKNOWN_CODEC = 7

# Color conversion library:
OPTION( USE_HDRTOOLS              "Clone, build and use HDRTools for color conversions"     FALSE )

# Use HM optimized RDO
OPTION( USE_HM_PCC_RDO            "Use HM optimized RDO (run \"./clear.sh all\" for each modification)" TRUE )

# PAPI profiling tools
OPTION( ENABLE_PAPI_PROFILING     "Enable PAPI profiling"                                   FALSE )

## CLANG-TIDY CHECK
set( CMAKE_CXX_STANDARD            14 )
set( CMAKE_CXX_STANDARD_REQUIRED   ON )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/dependencies/cmake")

INCLUDE( CheckCXXCompilerFlag )
CHECK_CXX_COMPILER_FLAG( "-std=c++14" COMPILER_SUPPORTS_CXX14 )
if (COMPILER_SUPPORTS_CXX14)
  SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14" )
ELSEIF (COMPILER_SUPPORTS_CXX11)
  SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
ELSEIF (COMPILER_SUPPORTS_CXX0X)
  SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x" )
ELSE ()
  MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

IF( ENABLE_PAPI_PROFILING )
  INCLUDE( ${CMAKE_SOURCE_DIR}/dependencies/cmake/papi.cmake ) 
ENDIF()
# SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-function -Wno-implicit -Wno-sign-compare -Wall -Wformat=0" )

SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )
IF (MSVC)
  MESSAGE(STATUS "MSVC OPTIONS")
  ADD_DEFINITIONS("/wd4244 /wd4267 /wd4996")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:tbb_debug.lib")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:tbb.lib")
ENDIF()

# Include external libraries
IF( USE_HMLIB_VIDEO_CODEC )
  INCLUDE( ${CMAKE_SOURCE_DIR}/dependencies/cmake/hm.cmake )
ENDIF()
IF( USE_JMLIB_VIDEO_CODEC )
  INCLUDE( ${CMAKE_SOURCE_DIR}/dependencies/cmake/jm_lib.cmake )
ENDIF()
IF( USE_JMAPP_VIDEO_CODEC )
  INCLUDE( ${CMAKE_SOURCE_DIR}/dependencies/cmake/jm_app.cmake )
ENDIF()
IF( USE_FFMPEG_VIDEO_CODEC )
  INCLUDE( ${CMAKE_SOURCE_DIR}/dependencies/cmake/ffmpeg.cmake )
ENDIF()
IF( USE_VTMLIB_VIDEO_CODEC )
  INCLUDE( ${CMAKE_SOURCE_DIR}/dependencies/cmake/vtm.cmake )
ENDIF()
IF( USE_SHMAPP_VIDEO_CODEC )
  INCLUDE( ${CMAKE_SOURCE_DIR}/dependencies/cmake/shm_app.cmake )
ENDIF()
IF( USE_HDRTOOLS )  
  INCLUDE( ${CMAKE_SOURCE_DIR}/dependencies/cmake/hdrtools.cmake )
ENDIF()

## CLANG FORMAT
INCLUDE( dependencies/cmake/clang.cmake )

ADD_SUBDIRECTORY(dependencies)
ADD_SUBDIRECTORY(source/lib/PccLibVideoDecoder)
ADD_SUBDIRECTORY(source/lib/PccLibVideoEncoder)
ADD_SUBDIRECTORY(source/lib/PccLibColorConverter)
ADD_SUBDIRECTORY(source/lib/PccLibBitstreamCommon)
ADD_SUBDIRECTORY(source/lib/PccLibBitstreamWriter)
ADD_SUBDIRECTORY(source/lib/PccLibBitstreamReader)
ADD_SUBDIRECTORY(source/lib/PccLibCommon)
ADD_SUBDIRECTORY(source/lib/PccLibDecoder)
ADD_SUBDIRECTORY(source/lib/PccLibEncoder)
ADD_SUBDIRECTORY(source/lib/PccLibMetrics)
ADD_SUBDIRECTORY(source/lib/PccLibConformance)

ADD_SUBDIRECTORY(source/app/PccAppParser)
ADD_SUBDIRECTORY(source/app/PccAppEncoder)
ADD_SUBDIRECTORY(source/app/PccAppDecoder)
ADD_SUBDIRECTORY(source/app/PccAppMetrics)
ADD_SUBDIRECTORY(source/app/PccAppConformance)
ADD_SUBDIRECTORY(source/app/PccAppVideoEncoder)
ADD_SUBDIRECTORY(source/app/PccAppVideoDecoder)
ADD_SUBDIRECTORY(source/app/PccAppColorConverter)
ADD_SUBDIRECTORY(source/app/PccAppNormalGenerator)
