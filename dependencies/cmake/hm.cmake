cmake_minimum_required(VERSION 3.17 FATAL_ERROR)

set(HM_VERSION "HM-16.20+SCM-8.8" CACHE STRING "HM git branch/tag to use")
set(HM_REPO_URL "https://vcgit.hhi.fraunhofer.de/jvet/HM.git" CACHE STRING "HM repository URL")
option(HM_SKIP_PATCH "Skip HM patch application (migration/bootstrap only)" OFF)
set(HM_DIR ${CMAKE_SOURCE_DIR}/dependencies/${HM_VERSION}/)
set(HM_LIB_SOURCE_DIR ${HM_DIR}/source/Lib)
message("Clone HM libraries: ${HM_LIB_SOURCE_DIR}")

if(NOT EXISTS "${HM_DIR}/.git")
  message("  - HM clone")
  execute_process(COMMAND git clone --depth 1 --branch ${HM_VERSION} ${HM_REPO_URL} ${HM_DIR}
                  RESULT_VARIABLE ret)
  if(NOT ${ret} EQUAL "0")
    message(FATAL_ERROR "Error during the HM git clone process. Check that git is well installed on your system.")
  endif()
endif()

if(NOT HM_SKIP_PATCH AND NOT EXISTS "${HM_DIR}/PATCHED")
  set(HM_PATCH_DIR ${CMAKE_SOURCE_DIR}/dependencies/hm-modification)
  if(USE_HM_PCC_RDO)
    set(HM_PATCH ${HM_PATCH_DIR}/${HM_VERSION}_with_RDO.patch)
  else()
    set(HM_PATCH ${HM_PATCH_DIR}/${HM_VERSION}.patch)
  endif()
  if(NOT EXISTS "${HM_PATCH}")
    message(FATAL_ERROR "HM patch file not found for HM_VERSION=${HM_VERSION}: ${HM_PATCH}")
  endif()
  message("  - HM patch: ${HM_PATCH}")
  execute_process(COMMAND git apply ${HM_PATCH} --whitespace=nowarn
                  WORKING_DIRECTORY ${HM_DIR}
                  RESULT_VARIABLE ret)
  if(NOT ${ret} EQUAL "0")
    message(FATAL_ERROR "Error during the HM patch process. Check patch compatibility with HM_VERSION=${HM_VERSION}.")
  endif()
  file(WRITE ${HM_DIR}/PATCHED "HM patched with: " ${HM_PATCH})
endif()

if(HM_SKIP_PATCH)
  message("  - HM patch skipped (HM_SKIP_PATCH=ON)")
  add_compile_definitions(PCC_HM_UNPATCHED=1)
elseif(HM_VERSION MATCHES "^HM-18\\.0")
  # HM-18.0 patches don't create pcc_hm namespace wrappers like HM-16.20 does
  # so we treat it as unpatched from a namespace perspective
  message("  - HM-18.0 detected (using unpatched namespace mode)")
  add_compile_definitions(PCC_HM_UNPATCHED=1)
endif()

function(add_hm_library module)
  file(GLOB cppSourceFiles "${HM_LIB_SOURCE_DIR}/${module}/*.cpp")
  file(GLOB cSourceFiles "${HM_LIB_SOURCE_DIR}/${module}/*.c")
  file(GLOB headerFiles "${HM_LIB_SOURCE_DIR}/${module}/*.h")
  add_library(${module} ${cppSourceFiles} ${cSourceFiles} ${headerFiles})
  set_property(TARGET ${module} PROPERTY CXX_CLANG_TIDY) # no clang-tidy
  add_library(VPCC::${module} ALIAS ${module})
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
  
if(EXISTS "${HM_LIB_SOURCE_DIR}/TLibVideoIO")
  add_hm_library(TLibVideoIO)
else()
  # HM-18.x moved video I/O and option parsing helpers into Utilities.
  add_library(TLibVideoIO
    ${HM_LIB_SOURCE_DIR}/Utilities/TVideoIOYuv.cpp
    ${HM_LIB_SOURCE_DIR}/Utilities/program_options_lite.cpp
    ${HM_LIB_SOURCE_DIR}/Utilities/TVideoIOYuv.h
    ${HM_LIB_SOURCE_DIR}/Utilities/program_options_lite.h)
  set_property(TARGET TLibVideoIO PROPERTY CXX_CLANG_TIDY) # no clang-tidy
  add_library(VPCC::TLibVideoIO ALIAS TLibVideoIO)
endif()
target_link_libraries(TLibVideoIO PUBLIC TLibCommon)
target_compile_features(TLibVideoIO PUBLIC cxx_std_11)
target_include_directories(TLibVideoIO PUBLIC "$<BUILD_INTERFACE:${HM_LIB_SOURCE_DIR}>")

add_hm_library(TLibDecoder)
target_link_libraries(TLibDecoder PUBLIC TLibCommon)
 
add_hm_library(TLibEncoder)
target_link_libraries(TLibEncoder PUBLIC TLibCommon TLibVideoIO )
