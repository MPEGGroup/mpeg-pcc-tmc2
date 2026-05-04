import argparse
import os
import re


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--dir", required=True, help="Results directory")
    parser.add_argument("-s", "--seq", required=True, help="Sequence name")
    parser.add_argument("-r", "--rate", required=True, help="Rate string (e.g. r01)")
    parser.add_argument("-f", "--frames", type=int, default=1, help="Frame count")
    args = parser.parse_args()

    # Find all frame metrics
    frame_files = [f for f in os.listdir(args.dir) if f.endswith(".decoded.ply.metric")]
    if not frame_files:
        raise FileNotFoundError(f"No frames found with logs")
    frame_results = []
    for f in frame_files:
        # extract frame base name (without .decoded.ply.metric)
        base = f.replace(".decoded.ply.metric", "")
        metric_path = os.path.join(args.dir, f"{base}.decoded.ply.metric")
        enc_bitstream_path = os.path.join(args.dir, f"{base}_enc_bitstream.txt")
        enc_log_path = os.path.join(args.dir, f"{base}.encoder.log")
        dec_log_path = os.path.join(args.dir, f"{base}.decoder.log")

        if not os.path.exists(metric_path):
            raise FileNotFoundError(f"Missing file: {metric_path}")
        if not os.path.exists(enc_bitstream_path):
            raise FileNotFoundError(f"Missing file: {enc_bitstream_path}")
        if not os.path.exists(enc_log_path):
            raise FileNotFoundError(f"Missing file: {enc_log_path}")
        if not os.path.exists(dec_log_path):
            raise FileNotFoundError(f"Missing file: {dec_log_path}")

        frame_result = {}
        metric_data = parse_metric_file(metric_path)
        frame_result.update(metric_data)
        bitstream_data = parse_bitstream_file(enc_bitstream_path)
        frame_result.update(bitstream_data)
        enc_log_data = parse_enc_log_file(enc_log_path)
        frame_result.update(enc_log_data)
        dec_log_data = parse_dec_log_file(dec_log_path)
        frame_result.update(dec_log_data)

        frame_results.append(frame_result)

    # Average per-frame results
    avg_result = {}
    avg_result["seq"] = args.seq
    avg_result["rate"] = args.rate
    for key in frame_results[0].keys():
        avg_result[key] = sum(fr[key] for fr in frame_results) / len(frame_results)

    # Define output order
    columns = [
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

    print(",".join(str(avg_result[col]) for col in columns))


if __name__ == "__main__":
    main()
