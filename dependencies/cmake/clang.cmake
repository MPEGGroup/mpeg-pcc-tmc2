


file( GLOB_RECURSE ALL_CXX_SOURCE_FILES source/*.cpp source/*.h )

# clang-format 
find_program(CLANG_FORMAT "clang-format")
if(CLANG_FORMAT)
  add_custom_target(
    clang-format
    COMMAND clang-format-15
    -i
    -style=file
    ${ALL_CXX_SOURCE_FILES}
    )
    message( "add CLANG_FORMAT command" )
else ()
message( "CLANG_FORMAT can't be found" )
endif()

# clang-tidy
find_program(CLANG_TIDY "clang-tidy")
if(CLANG_TIDY)
    set( INCLUDE_ARGS " -I${CMAKE_CURRENT_SOURCE_DIR}/source/dependencies/nanoflann \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/dependencies/libmd5  \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/dependencies/tbb/include  \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/dependencies/program-options-lite  \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include   \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/lib/PccLibBitstreamReader/include   \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/lib/PccLibBitstreamWriter/include   \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/lib/PccLibCommon/include   \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/lib/PccLibDecoder/include  \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/lib/PccLibEncoder/include   \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/lib/PccLibMetrics/include  \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/app/PccAppDecoder/include  \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/app/PccAppEncoder/include  \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/app/PccAppMetrics/include   \
    -I${CMAKE_CURRENT_SOURCE_DIR}/source/app/PccAppParser/include  " )
     
    add_custom_target(
      clang-tidy
      COMMAND clang-tidy
      -p ./build/  
      -header-filter=include/*.h
      -system-headers=0
      -fix 
      -fix-errors 
      ${ALL_CXX_SOURCE_FILES}
      --
      ${INCLUDE_ARGS}
      )
endif()