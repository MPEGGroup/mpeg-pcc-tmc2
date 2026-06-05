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

import argparse
import os
import re

COLUMN_ORDER: list[str] = [
    "length",  # Length variant (e.g. 1F or NF)
    "seq",  # Sequence name (e.g. plant)
    "rate",  # Rate string (e.g. r01)
    "total_Byte",  # Total size of the bitsteam (the entire file)
    "pos_Byte",  # Total size of V3C_GVD units
    "sh_Byte",  # Total size of sphericalHarmonic attribute in V3C_AVD units
    "rot_Byte",  # Total size of rotation attribute in V3C_AVD units
    "scale_Byte",  # Total size of scale attribute in V3C_AVD units
    "opacity_Byte",  # Total size of opacity attribute in V3C_AVD units
    "packed0_Byte",  # Total size of PVD units with PackedMapIndex 0
    "packed1_Byte",  # Total size of PVD units with PackedMapIndex 1
    "packed2_Byte",  # Total size of PVD units with PackedMapIndex 2
    "packed3_Byte",  # Total size of PVD units with PackedMapIndex 3
    "meta_Byte",  # Total size of VPS and AD units
    "RGB-PSNR",  # Average RGB PSNR across all frames
    "YUV-PSNR",  # Average YUV PSNR across all frames
    "YUV-SSIM",  # Average YUV SSIM across all frames
    "min RGB-PSNR",  # Minimum RGB PSNR across all frames
    "min YUV-PSNR",  # Minimum YUV PSNR across all frames
    "min YUV-SSIM",  # Minimum YUV SSIM across all frames
    "max RGB-PSNR",  # Maximum RGB PSNR across all frames
    "max YUV-PSNR",  # Maximum YUV PSNR across all frames
    "max YUV-SSIM",  # Maximum YUV SSIM across all frames
    "total_impT",  # Total importance score computation CPU time
    "total_encT",  # Total encoding CPU time
    "netto_encT",  # Total encoding CPU time excluding importance score computation (i.e. total_encT - total_impT)
    "total_decT",  # Total decoding CPU time
    "total_metT",  # Total metric computation CPU time
    "imp_MaxRSS",  # Maximum resident set size during importance score computation
    "enc_MaxRSS",  # Maximum resident set size during encoding
    "netto_enc_MaxRSS",  # Maximum resident set size during encoding excluding importance score computation
    "dec_MaxRSS",  # Maximum resident set size during decoding
    "met_MaxRSS",  # Maximum resident set size during metric computation
]

TIME_PATTERN = re.compile(
    r"user_time=(\d+\.\d+) wall_time=.+ max_rss_KB=(\d+)\s*"
)


def parse_bitstream_file(path: str) -> dict[str, float]:
    attr_bytes: dict[str, float] = {
        k: 0.0 for k in COLUMN_ORDER if k.endswith("_Byte") and k != "total_Byte"
    }
    attribute: str | None = None
    packed_map: int | None = None

    with open(path, "rt") as f:
        for line in f:
            line = line.strip()

            # Detect the attribute inside videoSubStream
            if m_attr := re.match(r"\[ *\d+- *\d+\] *attribute *(\w+)", line):
                attribute = m_attr.group(1)
                continue

            if m_pmap := re.match(
                r"\[ *\d+- *\d+\] +PackedMapIndex \.+ = (\d+) ", line
            ):
                packed_map = int(m_pmap.group(1))

            # Detect the end of the V3C unit
            if m_end := re.match(r"\[\s*(\d+)-\d+\] \} ~v3cUnit\((V3C_[A-Z]+)\)", line):
                size = float(m_end.group(1))
                v3c_unit_type = m_end.group(2)

                if v3c_unit_type in ["V3C_VPS", "V3C_AD"]:
                    key = "meta_Byte"
                elif v3c_unit_type == "V3C_GVD":
                    key = "pos_Byte"
                elif v3c_unit_type == "V3C_AVD":
                    assert attribute is not None

                    if attribute == "rotation":
                        key = "rot_Byte"
                    elif attribute == "sphericalHarmonic":
                        key = "sh_Byte"  # Difficult to distinguish between sh0 .. sh3 with the remappings
                    else:
                        key = f"{attribute}_Byte"
                elif v3c_unit_type == "V3C_PVD":
                    assert packed_map is not None
                    key = f"packed{packed_map}_Byte"
                else:
                    raise ValueError(f"Unexpected V3C unit type: {v3c_unit_type}")

                if key is not None:
                    attr_bytes[key] += size

    return attr_bytes


# The complexity information is in the error files
def parse_err_file(path: str, total_time: str, max_rss: str) -> dict[str, float]:
    # initialize all metrics with 0.0
    metrics: dict[str, float] = {total_time: 0.0, max_rss: 0.0}
    count = 0

    with open(path, "rt") as f:
        for line in f:
            if m := TIME_PATTERN.match(line):
                count += 1
                metrics[total_time] += float(m.group(1))
                metrics[max_rss] += float(m.group(2))

    if count != 1:
        raise ValueError(f"Expected exactly one line matching the time pattern in {path}, but found {count}")

    return metrics


def opt_parse_imp_err_file(path: str) -> dict[str, float]:
    if os.path.exists(path):
        return parse_err_file(path, "total_impT", "imp_MaxRSS")

    return {"total_impT": 0.0, "imp_MaxRSS": 0.0}


def parse_enc_err_file(path: str) -> dict[str, float]:
    return parse_err_file(path, "total_encT", "enc_MaxRSS")


def parse_dec_err_file(path: str) -> dict[str, float]:
    return parse_err_file(path, "total_decT", "dec_MaxRSS")


def parse_met_err_file(path: str) -> dict[str, float]:
    return parse_err_file(path, "total_metT", "met_MaxRSS")


def parse_metric_file(path: str) -> dict[str, float]:
    # initialize all metrics with 0.0
    metrics: dict[str, float] = {
        "RGB-PSNR": 0.0,
        "YUV-PSNR": 0.0,
        "YUV-SSIM": 0.0,
    }

    with open(path, "rt") as f:
        for line in f:
            line = line.strip()
            # quality metrics
            if line.startswith("Psnr RGB (avg)"):
                metrics["RGB-PSNR"] = float(line.split()[-1])
            elif line.startswith("Psnr YUV (avg)"):
                metrics["YUV-PSNR"] = float(line.split()[-1])
            elif line.startswith("SSIM (avg)"):
                metrics["YUV-SSIM"] = float(line.split()[-1])
    return metrics


def per_frame_results(
    results_dir: str, base: str, start_frame: int, frame_count: int
) -> list[dict[str, float]]:
    results: list[dict[str, float]] = []

    for i in range(start_frame, start_frame + frame_count):
        frame_path = os.path.join(results_dir, f"plys/{base}_{i:04d}.decoded.ply")
        metric_path = os.path.join(
            results_dir, f"metrics/{base}_{i:04d}.decoded.ply.metric"
        )
        met_err_path = os.path.join(results_dir, f"metrics/{base}_{i:04d}.metric.err")

        # Importance scores are caclculated only once for all rates
        rate_id = os.path.basename(results_dir)
        imp_base = base.replace(f"_{rate_id}", "")
        imp_err_path = os.path.join(
            os.path.dirname(results_dir),
            "importance_scores",
            f"{imp_base}_{i:04d}.importance.err",
        )

        for file in (frame_path, metric_path, met_err_path):
            if not os.path.exists(file):
                raise FileNotFoundError(f"Missing file: {file}")

        frame_results = parse_metric_file(metric_path)
        frame_results.update(parse_met_err_file(met_err_path))
        frame_results.update(opt_parse_imp_err_file(imp_err_path))
        results.append(frame_results)

    return results


def aggregate_per_frame_results(
    per_frame_results: list[dict[str, float]],
) -> dict[str, float]:
    aggregates: dict[str, float] = {}

    for key in per_frame_results[0].keys():
        if key.startswith("total_"):  # Total CPU time
            aggregates[key] = sum(f[key] for f in per_frame_results)
        elif key.endswith("_MaxRSS"):  # Maximum resident set size
            aggregates[key] = max(f[key] for f in per_frame_results)
        else:  # Quality metric
            aggregates[key] = sum(f[key] for f in per_frame_results) / len(
                per_frame_results
            )
            aggregates[f"min {key}"] = min(f[key] for f in per_frame_results)
            aggregates[f"max {key}"] = max(f[key] for f in per_frame_results)

    return aggregates


def per_rate_results(results_dir: str, base: str) -> dict[str, float]:
    enc_bitstream_path = os.path.join(results_dir, f"{base}.bin")
    enc_bitstream_txt_path = os.path.join(results_dir, f"{base}_enc_bitstream.txt")
    enc_err_path = os.path.join(results_dir, f"{base}.encoder.err")
    dec_err_path = os.path.join(results_dir, f"{base}.decoder.err")

    for file in (
        enc_bitstream_path,
        enc_bitstream_txt_path,
        enc_err_path,
        dec_err_path,
    ):
        if not os.path.exists(file):
            raise FileNotFoundError(f"Missing file: {file}")

    result: dict[str, float] = {
        "total_Byte": float(os.path.getsize(enc_bitstream_path)),
    }
    result.update(parse_bitstream_file(enc_bitstream_txt_path))
    result.update(parse_enc_err_file(enc_err_path))
    result.update(parse_dec_err_file(dec_err_path))
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--print-header-row",
        action="store_true",
        help="Print CSV header row and exit",
    )
    parser.add_argument("-d", "--results-dir", help="Results directory")
    parser.add_argument("-l", "--length", help="Length (1F or NF)")
    parser.add_argument("-s", "--seq", help="Sequence name")
    parser.add_argument("-r", "--rate", help="Rate string (e.g. r01)")
    parser.add_argument("-f", "--start-frame", type=int, help="Start frame number")
    parser.add_argument("-n", "--frame-count", type=int, help="Number of frames")
    return parser.parse_args()


def main(
    print_header_row: bool,
    results_dir: str,
    length: str,
    seq: str,
    rate: str,
    start_frame: int,
    frame_count: int,
) -> None:
    if print_header_row:
        print(", ".join(COLUMN_ORDER))
        return

    base = f"{length}_{seq}_{rate}"
    frame_results = per_frame_results(results_dir, base, start_frame, frame_count)
    result = aggregate_per_frame_results(frame_results)
    result.update(per_rate_results(results_dir, base))

    # Remember netto encoding complexity w/o importance score calculation
    result["netto_encT"] = result["total_encT"]
    result["netto_enc_MaxRSS"] = result["enc_MaxRSS"]

    # Add importance scoring complexity to the encoding complexity
    result["total_encT"] = result["total_encT"] + result.get("total_impT", 0.0)
    result["enc_MaxRSS"] = max(result["enc_MaxRSS"], result.get("imp_MaxRSS", 0.0))

    info: dict[str, str | float] = {"length": length, "seq": seq, "rate": rate}
    info.update(result)
    print(", ".join(str(info[col]) for col in COLUMN_ORDER))


if __name__ == "__main__":
    args = parse_args()
    main(**vars(args))
