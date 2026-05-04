#!/usr/bin/env python3

# The copyright in this software is being made available under the BSD
# License, included below. This software may be subject to other third party
# and contributor rights, including patent rights, and no such rights are
# granted under this license.
#
# Copyright (c) 2026-2026, ISO/IEC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the ISO/IEC nor the names of its contributors may
#    be used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.

from pathlib import Path
import subprocess
from sys import stderr, executable

ROOT_DIR = Path(__file__).parent.parent.resolve()


def main() -> None:
    files = files_in_repository()
    format_python_scripts(files)
    format_cpp_files(files)
    format_cmake_files(files)


def files_in_repository() -> list[Path]:
    result = run(["git", "ls-files"], capture_output=True, text=True)
    return [
        f
        for f in (Path(line) for line in result.stdout.splitlines())
        if not "dependencies" in f.parts
    ]


def format_python_scripts(files: list[Path]) -> None:
    run([find_executable("black")] + [f for f in files if f.suffix == ".py"])


def format_cpp_files(files: list[Path]) -> None:
    run(
        [find_executable("clang-format"), "-i"]
        + [f for f in files if f.suffix in (".cpp", ".h")]
    )


def format_cmake_files(files: list[Path]) -> None:
    run(
        [find_executable("cmake-format"), "-i"]
        + [f for f in files if f.name == "CMakeLists.txt"]
    )


# Prefer executables that are available within the Python virtual environment to avoid version mismatches
def find_executable(name: str) -> str:
    target = Path(executable).parent / name

    for candidate in (target, target.with_suffix(".exe")):
        if candidate.is_file():
            return str(candidate)

    return name


def run(
    args: list[str | Path], check=True, cwd=ROOT_DIR, **kwargs
) -> subprocess.CompletedProcess:
    dir_change = "" if cwd == ROOT_DIR else f"cd {cwd} && "
    print(f"> {dir_change}{' '.join(map(str, args))}", file=stderr)
    return subprocess.run(args, check=check, cwd=cwd, **kwargs)


if __name__ == "__main__":
    main()
