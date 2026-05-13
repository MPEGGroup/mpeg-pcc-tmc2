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

# Constants for CSV column ranges and Excel positions
BITRATES_RANGE = (3, 12, 11)  # start_idx, end_idx, start_col
METRICS_RANGE = (21, 24, 20)
TIMES_RSS_RANGE = (13, 21, 31)


def write_values(ws, csv_tokens, start_idx, end_idx, start_col, rate_id, seq_offset):
    """
    Write a range of CSV values into the worksheet.
    Converts to float if not empty, otherwise writes None.
    """
    for col, val in enumerate(csv_tokens[start_idx:end_idx]):
        ws.cell(
            row=seq_offset + rate_id,
            column=start_col + col,
            value=float(val) if val else None,
        )
        print("WRITING", val, "to", (seq_offset + rate_id, start_col + col))


def csv2xlsm(wb, file_path, sheet_name):
    """
    Read a CSV file and write its data into the given workbook.
    sheet_name determines the sheet that will be filled out.
    """
    try:
        ws = wb[sheet_name]
    except KeyError:
        print(f"[ERROR] Sheet '{sheet_name}' not found in workbook")
        exit()

    # Build index from sequence variant, e.g. ("1F", "plant"), to row number in the worksheet
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

    warned_sequences = set()
    with open(file_path, "rt") as fp:
        next(fp)  # skip header line
        for line in fp:
            csv_tokens = line.strip().split(",")
            seq_variant = (csv_tokens[0], csv_tokens[1])
            if seq_variant not in sequence_rows:
                # print a warning
                if seq_variant not in warned_sequences:
                    print(
                        f"[WARNING] Sequence '{' '.join(seq_variant)}' not found in the template!"
                    )
                    warned_sequences.add(seq_variant)
                continue
            seq_row_offset = sequence_rows[seq_variant]
            print("FOUND", " ".join(seq_variant))
            rate_id = int(csv_tokens[2].split("r")[-1]) - 1

            # Write different sections of the CSV into the worksheet
            print("WRITING BITRATES")
            write_values(ws, csv_tokens, *BITRATES_RANGE, rate_id, seq_row_offset)
            print("WRITING METRICS")
            write_values(ws, csv_tokens, *METRICS_RANGE, rate_id, seq_row_offset)
            print("WRITING TIMES")
            write_values(ws, csv_tokens, *TIMES_RSS_RANGE, rate_id, seq_row_offset)


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
        print(f"[ERROR] No anchor CSV file found: {args.anc_csv}")
        exit()

    # Process proposed CSV
    if os.path.isfile(args.pro_csv):
        csv2xlsm(wb, args.pro_csv, "Proposal.C1")
    else:
        print(f"[ERROR] No proposed CSV file found: {args.pro_csv}")
        exit()

    # Save the updated workbook
    wb.save(args.xlsm)
    print(f"[INFO] XLSM file saved: {args.xlsm}")


if __name__ == "__main__":
    main()
