cmake_minimum_required(VERSION 3.17 FATAL_ERROR)

project(TMC2 CXX C)
set(TMC2_VERSION_MAJOR 26)
set(TMC2_VERSION_MINOR 0)

# Video encoder and decoder library and application:
option(USE_JMAPP_VIDEO_CODEC "Use JM application to encode and decode video"
       TRUE) # JMAPP  = 0
option(USE_HMAPP_VIDEO_CODEC "Use HM application to encode and decode video"
       TRUE) # HMAPP  = 1
option(USE_SHMAPP_VIDEO_CODEC "Use SHM application to encode and decode video"
       FALSE) # SHMAPP = 2
option(USE_JMLIB_VIDEO_CODEC
       "Clone, build and use JM video encoder and decoder" FALSE) # JMLIB  = 3
option(USE_HMLIB_VIDEO_CODEC
       "Clone, build and use HM video encoder and decoder" TRUE) # HMLIB  = 4
option(USE_VTMLIB_VIDEO_CODEC
       "Clone, build and use VTM video encoder and decoder" FALSE) # VTMLIB = 5
option(USE_VVLIB_VIDEO_CODEC
       "Clone, build and use VVENC/VVDEC video encoder and decoder" FALSE
)# VVENC  = 6
option(USE_FFMPEG_VIDEO_CODEC
       "Clone, build and use FFMPEG video encoder and decoder" FALSE
)# FFMPEG = 7

# Color conversion library:
option(USE_HDRTOOLS "Clone, build and use HDRTools for color conversions" FALSE)

# Use HM optimized RDO
option(USE_HM_PCC_RDO "Use HM optimized RDO (re-run \"./clear.sh all\" )" TRUE)

# Multi-threading and profiling tools
option(ENABLE_TBB "Enable TBB mutli-threading" FALSE)
option(ENABLE_PAPI_PROFILING "Enable PAPI profiling" FALSE)

# trace and conformance modes
option(BITSTREAM_TRACE "Enable bitstream traces" TRUE)
option(CODEC_TRACE "Enable encoder and decoder process traces" FALSE)
option(SEI_TRACE "Enable SEI bitstream traces" FALSE)
option(CONFORMANCE_TRACE "Enable conformance traces" FALSE)

# include the v3c parser for conformance check
option(USE_V3C_CPP_CODER "Include v3c-cpp-coder subdirectory" FALSE)

if(NOT DEFINED CMAKE_CXX_EXTENSIONS)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(NOT DEFINED CMAKE_CXX_STANDARD)
  list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_23 have_cxx_std_23)

  if(${have_cxx_std_23} GREATER_EQUAL 0)
    set(CMAKE_CXX_STANDARD 23)
  else()
    set(CMAKE_CXX_STANDARD 20)
  endif()
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
                      "${CMAKE_SOURCE_DIR}/dependencies/cmake")

if(ENABLE_PAPI_PROFILING)
  include(${CMAKE_SOURCE_DIR}/dependencies/cmake/papi.cmake)
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
if(MSVC)
  message(STATUS "MSVC OPTIONS")
  add_definitions("/wd5105")
  if(ENABLE_TBB)
    set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:tbb_debug.lib")
    set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:tbb.lib")
  endif()
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Include external libraries
if(USE_HMLIB_VIDEO_CODEC)
  include(${CMAKE_SOURCE_DIR}/dependencies/cmake/hm.cmake)
endif()
if(USE_JMLIB_VIDEO_CODEC)
  include(${CMAKE_SOURCE_DIR}/dependencies/cmake/jm_lib.cmake)
endif()
if(USE_JMAPP_VIDEO_CODEC)
  include(${CMAKE_SOURCE_DIR}/dependencies/cmake/jm_app.cmake)
endif()
if(USE_FFMPEG_VIDEO_CODEC)
  include(${CMAKE_SOURCE_DIR}/dependencies/cmake/ffmpeg.cmake)
endif()
if(USE_VTMLIB_VIDEO_CODEC)
  include(${CMAKE_SOURCE_DIR}/dependencies/cmake/vtm.cmake)
endif()
if(USE_SHMAPP_VIDEO_CODEC)
  include(${CMAKE_SOURCE_DIR}/dependencies/cmake/shm_app.cmake)
endif()
if(USE_VVLIB_VIDEO_CODEC)
  include(${CMAKE_SOURCE_DIR}/dependencies/cmake/vvlib.cmake)
endif()
if(USE_HDRTOOLS)
  include(${CMAKE_SOURCE_DIR}/dependencies/cmake/hdrtools.cmake)
endif()

# CLANG FORMAT
include(dependencies/cmake/clang.cmake)

add_subdirectory(dependencies)
add_subdirectory(source/lib/PccLibVideoDecoder)
add_subdirectory(source/lib/PccLibVideoEncoder)
add_subdirectory(source/lib/PccLibColorConverter)
add_subdirectory(source/lib/PccLibBitstreamCommon)
add_subdirectory(source/lib/PccLibBitstreamWriter)
add_subdirectory(source/lib/PccLibBitstreamReader)
add_subdirectory(source/lib/PccLibCommon)
add_subdirectory(source/lib/PccLibDecoder)
add_subdirectory(source/lib/PccLibEncoder)
add_subdirectory(source/lib/PccLibMetrics)
add_subdirectory(source/lib/PccLibConformance)
add_subdirectory(source/lib/PccLibGSEncoder)
add_subdirectory(source/lib/PccLibAdaptation)

add_subdirectory(source/app/PccAppParser)
add_subdirectory(source/app/PccAppEncoder)
add_subdirectory(source/app/PccAppDecoder)
add_subdirectory(source/app/PccAppMetrics)
add_subdirectory(source/app/PccAppConformance)
add_subdirectory(source/app/PccAppVideoEncoder)
add_subdirectory(source/app/PccAppVideoDecoder)
add_subdirectory(source/app/PccAppColorConverter)
add_subdirectory(source/app/PccAppNormalGenerator)
add_subdirectory(source/app/PccGs4AppEncoder)

if(USE_V3C_CPP_CODER)
  set(v3c_coder_BUILD_TESTING OFF)
  add_subdirectory(v3c-cpp-coder)
endif()
