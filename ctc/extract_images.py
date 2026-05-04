"""
Convert a raw 8-bit RGB 4:4:4 stream (.rgb) into a sequence of PNG views.
Supports both interleaved (R,G,B per pixel) and planar (all R, then all G, then all B) layouts.

Contributor:
-----------
    alexandre.zaghetto@sony.com

Date:
-----
    2025-08-28
"""

import os
import argparse
import numpy as np
import imageio.v2 as imageio


def parse_arguments():
    p = argparse.ArgumentParser(
        description="Convert raw 8-bit RGB 4:4:4 data into PNG views."
    )
    p.add_argument(
        "--input", required=True, type=str, help="Path to the input .rgb file"
    )
    p.add_argument(
        "--output", required=True, type=str, help="Output directory for PNG views"
    )
    p.add_argument("--width", type=int, required=True, help="Frame width in pixels")
    p.add_argument("--height", type=int, required=True, help="Frame height in pixels")
    p.add_argument(
        "--planar",
        type=int,
        help="Set to 1 if input is planar RGB (all R, then G, then B)",
    )
    return p.parse_args()


def read_rgb444(filename: str, width: int, height: int, planar: bool) -> np.ndarray:
    """
    Read a raw RGB444 byte stream and reshape into (N, H, W, 3).

    planar=0 → interleaved [R,G,B,R,G,B,...]
    planar=1  → planar [all R][all G][all B] per frame
    """
    frame_bytes = width * height * 3
    with open(filename, "rb") as f:
        data = np.frombuffer(f.read(), dtype=np.uint8)

    if data.size % frame_bytes != 0:
        raise ValueError(
            f"Input size {data.size} bytes is not a multiple of frame size {frame_bytes} bytes."
        )

    n_frames = data.size // frame_bytes
    if n_frames == 0:
        raise ValueError("No full frames found in input.")

    if planar == 1:
        views_list = []
        for n in range(n_frames):
            offset = n * frame_bytes
            R = data[offset : offset + width * height].reshape(height, width)
            G = data[offset + width * height : offset + 2 * width * height].reshape(
                height, width
            )
            B = data[offset + 2 * width * height : offset + 3 * width * height].reshape(
                height, width
            )
            frame = np.stack((R, G, B), axis=-1)  # H x W x 3
            views_list.append(frame)
        views = np.stack(views_list, axis=0)
    else:
        views = data.reshape(n_frames, height, width, 3)

    return views


if __name__ == "__main__":
    args = parse_arguments()

    os.makedirs(args.output, exist_ok=True)

    views = read_rgb444(args.input, args.width, args.height, args.planar)
    print(f"Frames detected: {len(views)}")

    for i, view in enumerate(views):
        out_path = os.path.join(args.output, f"{i:04d}.png")
        imageio.imwrite(out_path, view)
        print(f"Saved {out_path}")

    print("Done.")
