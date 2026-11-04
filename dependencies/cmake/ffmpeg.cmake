cmake_minimum_required(VERSION 3.5)

# Locate the FFmpeg libraries (libavcodec / libavutil / libavformat) used by the
# FFMPEG video codec backend (encoding is done through the "libkvazaar" encoder,
# so the FFmpeg build must be configured with --enable-libkvazaar).
#
# The following cache variables can be set by the user to point at a custom
# FFmpeg installation:
#   FFMPEG_DIR      : root of a prebuilt FFmpeg (expects include/ and lib/)
#   FFMPEG_DEV_DIR  : Windows dev package root (expects include/ and lib/)
#
# This module exports (consumed by the Pcc*VideoEncoder/Decoder CMakeLists):
#   FFMPEG_INCLUDE, AVCODEC_LIBRARY, AVUTIL_LIBRARY, AVFORMAT_LIBRARY
#   KVAZAAR_INCLUDE, KVAZAAR_LIBRARY (native Kvazaar API used for encoding)

set(FFMPEG_DIR "" CACHE PATH "Root directory of a prebuilt FFmpeg installation")
set(FFMPEG_DEV_DIR "${FFMPEG_DIR}" CACHE PATH "FFmpeg dev package root (Windows)")

set(_FFMPEG_HINT_INCLUDE "")
set(_FFMPEG_HINT_LIB "")
if(FFMPEG_DIR)
  set(_FFMPEG_HINT_INCLUDE ${FFMPEG_DIR}/include)
  set(_FFMPEG_HINT_LIB ${FFMPEG_DIR}/lib)
endif()

# Prefer pkg-config when no explicit FFMPEG_DIR is given.
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND AND NOT FFMPEG_DIR)
  pkg_check_modules(PC_AVCODEC QUIET libavcodec)
  pkg_check_modules(PC_AVUTIL QUIET libavutil)
  pkg_check_modules(PC_AVFORMAT QUIET libavformat)
  pkg_check_modules(PC_KVAZAAR QUIET kvazaar)
endif()

find_path(FFMPEG_INCLUDE
          NAMES libavcodec/avcodec.h
          HINTS ${_FFMPEG_HINT_INCLUDE} ${PC_AVCODEC_INCLUDE_DIRS})

find_library(AVCODEC_LIBRARY
             NAMES avcodec
             HINTS ${_FFMPEG_HINT_LIB} ${PC_AVCODEC_LIBRARY_DIRS})

find_library(AVUTIL_LIBRARY
             NAMES avutil
             HINTS ${_FFMPEG_HINT_LIB} ${PC_AVUTIL_LIBRARY_DIRS})

find_library(AVFORMAT_LIBRARY
             NAMES avformat
             HINTS ${_FFMPEG_HINT_LIB} ${PC_AVFORMAT_LIBRARY_DIRS})

# Kvazaar is used directly (native API) for encoding, since FFmpeg's libkvazaar
# wrapper is limited to 8-bit yuv420p. It may live under lib or lib64.
find_path(KVAZAAR_INCLUDE
          NAMES kvazaar.h
          HINTS ${_FFMPEG_HINT_INCLUDE} ${PC_KVAZAAR_INCLUDE_DIRS})

find_library(KVAZAAR_LIBRARY
             NAMES kvazaar
             HINTS ${_FFMPEG_HINT_LIB} ${FFMPEG_DIR}/lib64 ${PC_KVAZAAR_LIBRARY_DIRS})

if(NOT FFMPEG_INCLUDE
   OR NOT AVCODEC_LIBRARY
   OR NOT AVUTIL_LIBRARY
   OR NOT AVFORMAT_LIBRARY)
  message(FATAL_ERROR
          "USE_FFMPEG_VIDEO_CODEC is ON but FFmpeg could not be found. "
          "Install the FFmpeg development packages (libavcodec/libavutil/libavformat) "
          "or set -DFFMPEG_DIR=/path/to/ffmpeg.")
endif()

if(NOT KVAZAAR_INCLUDE OR NOT KVAZAAR_LIBRARY)
  message(FATAL_ERROR
          "USE_FFMPEG_VIDEO_CODEC is ON but Kvazaar could not be found. "
          "Build it (dependencies/ffmpeg-kvazaar/build_ffmpeg_kvazaar.sh) with 10-bit "
          "support (KVZ_BIT_DEPTH=10) or set -DFFMPEG_DIR=/path/to/prefix.")
endif()

message(STATUS "FFmpeg include: ${FFMPEG_INCLUDE}")
message(STATUS "FFmpeg avcodec: ${AVCODEC_LIBRARY}")
message(STATUS "FFmpeg avutil:  ${AVUTIL_LIBRARY}")
message(STATUS "FFmpeg avformat:${AVFORMAT_LIBRARY}")
message(STATUS "Kvazaar include:${KVAZAAR_INCLUDE}")
message(STATUS "Kvazaar library:${KVAZAAR_LIBRARY}")
