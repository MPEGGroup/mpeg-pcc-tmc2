#!/bin/bash
# Local provisioning of Kvazaar + a minimal FFmpeg (shared libs) for the
# USE_FFMPEG_VIDEO_CODEC backend. Everything is installed under a local prefix
# so no root privileges are required. Safe to delete the whole directory to undo.
#
# Kvazaar is built with 10-bit support (KVZ_BIT_DEPTH=10) and driven through its
# native API for encoding, because FFmpeg's libkvazaar wrapper is limited to
# 8-bit yuv420p. FFmpeg here is only used to decode the reconstructed HEVC.
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREFIX="${ROOT}/install"
JOBS="$(grep -c ^processor /proc/cpuinfo)"
mkdir -p "${ROOT}"
cd "${ROOT}"

export PKG_CONFIG_PATH="${PREFIX}/lib/pkgconfig:${PREFIX}/lib64/pkgconfig:${PKG_CONFIG_PATH}"

echo "========================================"
echo " 1/2  Building Kvazaar (10-bit)"
echo "========================================"
if [ ! -d kvazaar/.git ]; then
  git clone --depth 1 https://github.com/ultravideo/kvazaar.git
fi
# KVZ_BIT_DEPTH=10 makes kvz_pixel a uint16_t so Main10 / lossless 10-bit
# geometry can be encoded. The backend defines the same macro when it includes
# kvazaar.h, keeping the pixel type in sync with the library.
cmake -S kvazaar -B kvazaar/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
      -DBUILD_SHARED_LIBS=ON \
      -DBUILD_TESTS=OFF \
      -DCMAKE_C_FLAGS="-DKVZ_BIT_DEPTH=10"
cmake --build kvazaar/build -j "${JOBS}"
cmake --install kvazaar/build

# The backend links Kvazaar directly; fail early with a clear message if the
# install did not register with pkg-config.
if ! pkg-config --exists kvazaar; then
  echo "ERROR: pkg-config cannot find kvazaar after install." >&2
  echo "       Expected kvazaar.pc under ${PREFIX}/lib(64)/pkgconfig" >&2
  echo "       PKG_CONFIG_PATH=${PKG_CONFIG_PATH}" >&2
  exit 1
fi
echo "Found kvazaar $(pkg-config --modversion kvazaar) via pkg-config."

echo "========================================"
echo " 2/2  Building FFmpeg (HEVC decoder only)"
echo "========================================"
if [ ! -d FFmpeg/.git ]; then
  git clone --depth 1 --branch n6.1 https://github.com/FFmpeg/FFmpeg.git
fi
cd FFmpeg
./configure \
  --prefix="${PREFIX}" \
  --enable-shared --disable-static \
  --disable-x86asm \
  --disable-doc --disable-programs \
  --enable-decoder=hevc --enable-parser=hevc
make -j "${JOBS}"
make install

echo "========================================"
echo " DONE. Prefix: ${PREFIX}"
echo "========================================"
ls -1 "${PREFIX}/lib" 2>/dev/null | grep -E 'libav' || true
ls -1 "${PREFIX}/lib64" 2>/dev/null | grep -E 'libkvazaar' || true