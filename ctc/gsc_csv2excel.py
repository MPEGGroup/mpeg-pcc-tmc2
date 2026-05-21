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

import openpyxl
import os
import argparse
import re

TITLE_MAPPING = {
    "total_Byte": ("bitstream size [bytes]", "Total"),
    "pos_Byte": ("bitstream size [bytes]", "position"),
    "sh0_Byte": ("bitstream size [bytes]", "sh0"),
    "sh1_Byte": ("bitstream size [bytes]", "sh1"),
    "sh2_Byte": ("bitstream size [bytes]", "sh2"),
    "sh3_Byte": ("bitstream size [bytes]", "sh3"),
    "rot_Byte": ("bitstream size [bytes]", "rotation"),
    "scale_Byte": ("bitstream size [bytes]", "scaling"),
    "opacity_Byte": ("bitstream size [bytes]", "opacity"),
    "total_encT": ("Encoding time [s]", "Total"),
    "pos_encT": ("Encoding time [s]", "Geometry"),
    "att_encT": ("Encoding time [s]", "Attributes"),
    "total_decT": ("Decoding time [s]", "Total"),
    "pos_decT": ("Decoding time [s]", "Geometry"),
    "att_decT": ("Decoding time [s]", "Attributes"),
    "enc_MaxRSS": ("Max RSS [KB]", "encoder"),
    "dec_MaxRSS": ("Max RSS [KB]", "decoder"),
    "RGB-PSNR": ("Average objective quality", "RGB-PSNR"),
    "YUV-PSNR": ("Average objective quality", "YUV-PSNR"),
    "YUV-SSIM": ("Average objective quality", "YUV-SSIM"),
    "YUV-IVSSIM": ("Average objective quality", "Custom A"),
    "min RGB-PSNR": ("Minimum objective quality", "RGB-PSNR"),
    "min YUV-PSNR": ("Minimum objective quality", "YUV-PSNR"),
    "min YUV-SSIM": ("Minimum objective quality", "YUV-SSIM"),
    "max RGB-PSNR": ("Maximum objective quality", "RGB-PSNR"),
    "max YUV-PSNR": ("Maximum objective quality", "YUV-PSNR"),
    "max YUV-SSIM": ("Maximum objective quality", "YUV-SSIM"),
}


def write_values(ws, row, title_columns, measurement):
    """
    Write a range of CSV values into the worksheet.
    Converts to float if not empty, otherwise writes None.
    """
    for key, value in measurement.items():
        if title := TITLE_MAPPING.get(key):
            if column := title_columns.get(title):
                ws.cell(row=row, column=column, value=float(value) if value else None)
                print(f"    Writing {key} == {value} to ({row}, {column})")
            else:
                print(f"    WARNING: Could not map {title} to a column")
        else:
            print(f"    WARNING: Could not map {key} to a title")


def build_sequence_index(ws):
    """
    Build index from sequence variant, e.g. ("1F", "plant"), to row number in the worksheet
    """
    sequence_rows = {
        (
            ws.cell(row=cell.row, column=2).value,
            ws.cell(row=cell.row, column=3).value,
        ): cell.row
        for cell in ws["B"]
        if cell.value is not None
    }
    for key, value in sequence_rows.items():
        print(f"Sequence variant {' '.join(key)} is at row {value}")

    return sequence_rows


def build_title_index(ws):
    """
    Build index from title and sub-title, e.g. ("bitstream size [bytes]", "sh3"), to column number in the worksheet
    """
    title = None
    title_columns = {}

    for col in ws.iter_cols():
        if not col[1].value:
            continue
        if col[0].value:
            title = col[0].value
        if title is None:
            continue

        sub_title = col[1].value
        full_title = (title, sub_title)
        assert not full_title in title_columns
        title_columns[full_title] = col[0].column

    for key, value in title_columns.items():
        print(f'Title "{key}" is at column {value}')

    return title_columns


def csv2xlsm(wb, file_path, sheet_name):
    """
    Read a CSV file and write its data into the given workbook.
    sheet_name determines the sheet that will be filled out.
    """
    try:
        ws = wb[sheet_name]
    except KeyError:
        raise RuntimeError(f"ERROR: Sheet '{sheet_name}' not found in workbook")

    sequence_rows = build_sequence_index(ws)
    title_columns = build_title_index(ws)

    with open(file_path, "rt") as fp:
        keys = [s.strip() for s in next(fp).strip().split(",")]
        assert keys[:3] == ["length", "seq", "rate"]

        for line in fp:
            values = [s.strip() for s in line.strip().split(",")]
            assert len(keys) == len(values)
            seq_variant = (values[0], values[1])

            rate = values[2]
            m_rate = re.match(r"r(\d+)", rate)
            assert m_rate
            rate_idx = int(m_rate[1]) - 1
            assert rate_idx >= 0

            measurement = {keys[i]: values[i] for i in range(3, len(keys))}

            if seq_row_offset := sequence_rows.get(seq_variant):
                print(' '.join(values[:3]))
                write_values(ws, seq_row_offset + rate_idx, title_columns, measurement)
            else:
                print(f"WARNING: Sequence '{' '.join(seq_variant)}' not in template.")


def main():
    parser = argparse.ArgumentParser(description="Generate XLSM file from CSV data.")
    parser.add_argument(
        "-t", "--template", required=True, help="Path to XLSM template file"
    )
    parser.add_argument(
        "-a", "--anc_csv", required=True, help="Path to anchor CSV file"
    )
    parser.add_argument(
        "-p", "--pro_csv", required=True, help="Path to proposed CSV file"
    )
    parser.add_argument("-x", "--xlsm", required=True, help="Path to output XLSM file")
    args = parser.parse_args()

    # Load the template workbook with VBA macros preserved
    wb = openpyxl.load_workbook(args.template, read_only=False, keep_vba=True)

    # Process anchor CSV
    if os.path.isfile(args.anc_csv):
        csv2xlsm(wb, args.anc_csv, "Anchor.C1")
    else:
        raise RuntimeError(f"ERROR: No anchor CSV file found: {args.anc_csv}")

    # Process proposed CSV
    if os.path.isfile(args.pro_csv):
        csv2xlsm(wb, args.pro_csv, "Proposal.C1")
    else:
        raise RuntimeError(f"ERROR: No proposed CSV file found: {args.pro_csv}")

    # Save the updated workbook
    wb.save(args.xlsm)
    print(f"XLSM file saved: {args.xlsm}")


if __name__ == "__main__":
    main()
