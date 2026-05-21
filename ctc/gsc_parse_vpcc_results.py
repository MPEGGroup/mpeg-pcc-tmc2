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

COLUMN_ORDER = [
    "length",
    "seq",
    "rate",
    "total_Byte",
    "pos_Byte",
    "sh0_Byte",
    "sh1_Byte",
    "sh2_Byte",
    "sh3_Byte",
    "rot_Byte",
    "scale_Byte",
    "opacity_Byte",
    "meta_Byte",
    "total_encT",
    "pos_encT",
    "att_encT",
    "total_decT",
    "pos_decT",
    "att_decT",
    "enc_MaxRSS",
    "dec_MaxRSS",
    "RGB-PSNR",
    "YUV-PSNR",
    "YUV-SSIM",
    "YUV-IVSSIM",
    "RGB-LPIPS",
    "GT-RGB-PSNR",
    "renT",
]


def parse_bitstream_file(path):
    attr_bytes = {
        "pos_Byte": 0,
        "sh0_Byte": 0,
        "sh1_Byte": 0,
        "sh2_Byte": 0,
        "sh3_Byte": 0,
        "rot_Byte": 0,
        "scale_Byte": 0,
        "opacity_Byte": 0,
        "meta_Byte": 0,
        "total_Byte": 0,
    }

    v3c_unit_attr = None
    attr_count = {}

    attribute_name_remap = {
        "position": "pos",
        "opacity": "opacity",
        "sphericalHarmonic": "sh",
        "scale": "scale",
        "rotation": "rot",
    }
    with open(path, "rt") as f:
        for line in f:
            line = line.strip()

            # Detect the attribute inside videoSubStream
            m_attr = re.match(r"\[ *\d+- *\d+\] *attribute *(\w+)", line)
            if m_attr:
                if v3c_unit_attr is not None:
                    raise "ERR, two attributes detected in one v3cUnit"
                v3c_unit_attr = m_attr.group(1)
                continue

            # Detect the end of the V3C unit
            m_end = re.match(r"\[\s*(\d+)-\d+\] \} ~v3cUnit", line)
            if m_end:
                size = int(m_end.group(1))

                if v3c_unit_attr is None:
                    # Position is encoded differently, it has no v3c_unit_attr
                    v3c_unit_attr = "position"

                if v3c_unit_attr not in attribute_name_remap:
                    attr_name = "meta"
                else:
                    attr_name = attribute_name_remap[v3c_unit_attr]

                # Increment attribute count
                if attr_name not in attr_count:
                    attr_count[attr_name] = 0
                attr_count[attr_name] += 1

                # Log size
                if attr_name == "sh":
                    sh_count = attr_count[attr_name]
                    if sh_count in (1,):
                        sh_name = f"sh0"
                    elif sh_count in (2, 3, 4):
                        sh_name = f"sh1"
                    elif sh_count in (5, 6, 7, 8, 9):
                        sh_name = f"sh2"
                    elif sh_count in (10, 11, 12, 13, 14, 15, 16):
                        sh_name = f"sh3"
                    key = f"{sh_name}_Byte"
                else:
                    key = f"{attr_name}_Byte"
                attr_bytes[key] += size
                attr_bytes["total_Byte"] += size

                # Reset for next unit
                v3c_unit_attr = None

    return attr_bytes


def parse_enc_log_file(path):
    # initialize all metrics with 0.0
    metrics = {
        "total_encT": 0.0,
        "pos_encT": 0.0,
        "att_encT": 0.0,
        "enc_MaxRSS": 0.0,
    }

    with open(path, "rt") as f:
        for line in f:
            line = line.strip()
            # timing values
            if "(user.self):" in line:
                metrics["total_encT"] = float(line.split()[-2])
            if line.startswith("GeometryVideo"):
                metrics["pos_encT"] = float(line.split()[-2]) / 1000000
            if line.startswith("Execution"):
                metrics["att_encT"] += float(line.split()[-2]) / 1000000
            # memory usage
            for key in ["memory:"]:
                if key in line:
                    metrics["enc_MaxRSS"] = float(line.split()[-2])
    return metrics


def parse_dec_log_file(path):
    # initialize all metrics with 0.0
    metrics = {
        "total_decT": 0.0,
        "pos_decT": 0.0,
        "att_decT": 0.0,
        "dec_MaxRSS": 0.0,
    }

    with open(path, "rt") as f:
        for line in f:
            line = line.strip()
            # timing values
            if "(user.self):" in line:
                metrics["total_decT"] = float(line.split()[-2])
            if line.startswith("Execution") and ("Geometry" in line):
                metrics["pos_decT"] = float(line.split()[-2]) / 1000000
            if line.startswith("Execution") and ("Attribute" in line):
                metrics["att_decT"] += float(line.split()[-2]) / 1000000
            # memory usage
            for key in ["memory:"]:
                if key in line:
                    metrics["dec_MaxRSS"] = float(line.split()[-2])
    return metrics


def parse_metric_file(path):
    # initialize all metrics with 0.0
    metrics = {
        "RGB-PSNR": 0.0,
        "YUV-PSNR": 0.0,
        "YUV-SSIM": 0.0,
        "YUV-IVSSIM": 0.0,
        "RGB-LPIPS": 0.0,
        "GT-RGB-PSNR": 0.0,
        "renT": 0.0,
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
            elif line.startswith("IVSSIM YUV"):
                metrics["YUV-IVSSIM"] = float(line.split()[-1])
            elif line.startswith("IVSSIM RGB"):
                metrics["RGB-IVSSIM"] = float(line.split()[-1])
            elif line.startswith("RGB-LPIPS"):
                metrics["RGB-LPIPS"] = float(line.split()[-1])
            elif line.startswith("GT-RGB-PSNR"):
                metrics["GT-RGB-PSNR"] = float(line.split()[-1])
    return metrics


def per_frame_results(results_dir: str, base: str, start_frame: int, frame_count: int):
    results = []

    for i in range(start_frame, start_frame + frame_count):
        frame_path = os.path.join(results_dir, f"plys/{base}_{i:04d}.decoded.ply")
        metric_path = os.path.join(
            results_dir, f"metrics/{base}_{i:04d}.decoded.ply.metric"
        )

        for file in (frame_path, metric_path):
            if not os.path.exists(file):
                raise FileNotFoundError(f"Missing file: {file}")

        results.append(parse_metric_file(metric_path))

    return results


def average_per_frame_results(per_frame_results):
    average = {}

    for key in per_frame_results[0].keys():
        average[key] = sum(f[key] for f in per_frame_results) / len(per_frame_results)

    return average


def per_rate_results(results_dir: str, base: str):
    enc_log_path = os.path.join(results_dir, f"{base}.encoder.log")
    dec_log_path = os.path.join(results_dir, f"{base}.decoder.log")
    enc_bitstream_path = os.path.join(results_dir, f"{base}_enc_bitstream.txt")

    for file in (enc_bitstream_path, enc_log_path, dec_log_path):
        if not os.path.exists(file):
            raise FileNotFoundError(f"Missing file: {file}")

    result = {}
    bitstream_data = parse_bitstream_file(enc_bitstream_path)
    result.update(bitstream_data)
    enc_log_data = parse_enc_log_file(enc_log_path)
    result.update(enc_log_data)
    dec_log_data = parse_dec_log_file(dec_log_path)
    result.update(dec_log_data)
    return result


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--results-dir", required=True, help="Results directory")
    parser.add_argument("-l", "--length", required=True, help="Length (1F or NF)")
    parser.add_argument("-s", "--seq", required=True, help="Sequence name")
    parser.add_argument("-r", "--rate", required=True, help="Rate string (e.g. r01)")
    parser.add_argument(
        "-f", "--start-frame", required=True, type=int, help="Start frame number"
    )
    parser.add_argument(
        "-n", "--frame-count", required=True, type=int, help="Number of frames"
    )
    return parser.parse_args()


def main(
    results_dir: str,
    length: str,
    seq: str,
    rate: str,
    start_frame: int,
    frame_count: int,
):
    base = f"{length}_{seq}_{rate}"
    result = {
        "length": length,
        "seq": seq,
        "rate": rate,
    }
    frame_results = per_frame_results(results_dir, base, start_frame, frame_count)
    result.update(average_per_frame_results(frame_results))
    result.update(per_rate_results(results_dir, base))
    print(",".join(str(result[col]) for col in COLUMN_ORDER))


if __name__ == "__main__":
    args = parse_args()
    main(**vars(args))
