# The copyright in this software is being made available under the BSD
# Licence, included below.  This software may be subject to other third
# party and contributor rights, including patent rights, and no such
# rights are granted under this licence.
#
# Copyright (c) 2026, ISO/IEC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
# * Neither the name of the ISO/IEC nor the names of its contributors
#   may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from argparse import ArgumentParser, Namespace
from pathlib import Path


def parse_args() -> Namespace:
    parser = ArgumentParser(description="Calculate the complexity of anchors")
    parser.add_argument(
        "--results-file",
        "-i",
        type=Path,
        help="CSV file containing the objective results of the experiment",
        default=Path(__file__).parent / "anchor_results" / "VPCC-GS.csv",
    )
    return parser.parse_args()


def main(results_file: Path) -> None:
    results = load_results(results_file)
    print_cpu_time(results)
    print()
    print_max_rss(results)


_CPU_TIME_KEYS = ["total_impT", "total_encT", "netto_encT", "total_decT", "total_metT"]


def print_cpu_time(results: list[dict[str, str]]) -> None:
    super_total = 0.0
    variant_super_totals = dict[str, float]()

    for key in _CPU_TIME_KEYS:
        print(f"=== {key} ===")
        total = 0.0
        variant_totals = dict[str, float]()

        for result in results:
            value = float(result[key])
            total += value
            id = variant_id(result)
            variant_totals[id] = variant_totals.get(id, 0.0) + value

        if key.startswith("total_"):
            super_total += total

            for id, sub_total in variant_totals.items():
                variant_super_totals[id] = variant_super_totals.get(id, 0.0) + sub_total

        print(f"Total: {total/3600:.2f} h")

        for id, sub_total in variant_totals.items():
            print(f"{id}: {sub_total/3600:.2f} h")

    print(f"=== Total CPU Time ===")
    print(f"{super_total/3600:.2f} h")

    for id, sub_total in variant_super_totals.items():
        print(f"{id}: {sub_total/3600:.2f} h")


_MAX_RSS_KEYS = [
    "imp_MaxRSS",
    "enc_MaxRSS",
    "netto_enc_MaxRSS",
    "dec_MaxRSS",
    "met_MaxRSS",
]

_MB = 1048576


def print_max_rss(results: list[dict[str, str]]) -> None:
    super_total = 0.0
    variant_super_totals = dict[str, float]()

    for key in _MAX_RSS_KEYS:
        print(f"=== {key} ===")
        total = 0.0
        variant_totals = dict[str, float]()

        for result in results:
            value = float(result[key])
            total = max(total, value)
            id = variant_id(result)
            variant_totals[id] = max(variant_totals.get(id, 0.0), value)

        super_total = max(super_total, total)

        for id, sub_total in variant_totals.items():
            variant_super_totals[id] = max(variant_super_totals.get(id, 0.0), sub_total)

        print(f"Total: {total/_MB:.2f} GB")

        for id, sub_total in variant_totals.items():
            print(f"{id}: {sub_total/_MB:.2f} GB")

    print(f"=== Total MaxRSS ===")
    print(f"{super_total/_MB:.2f} GB")

    for id, sub_total in variant_super_totals.items():
        print(f"{id}: {sub_total/_MB:.2f} GB")


def load_results(file: Path) -> list[dict[str, str]]:
    with file.open(encoding="utf8") as stream:
        lines = stream.readlines()
        keys = [k.strip() for k in lines[0].split(",")]
        return [dict(zip(keys, [k.strip() for k in l.split(",")])) for l in lines[1:]]


def variant_id(result: dict[str, str]) -> str:
    return result["length"] + " -- " + ("CW" if result["rate"] == "lossless" else "C1")


if __name__ == "__main__":
    args = parse_args()
    main(**vars(args))
