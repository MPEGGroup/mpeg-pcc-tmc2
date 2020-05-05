add_subdirectory( tbb )
IF( NOT USE_HM_VIDEO_CODEC )
  add_subdirectory( PccLibHevcParser )
ENDIF()