# Conducting V-PCC GS experiments

## Introduction

This is a guide to generate the V-PCC anchor [[WG 5 N 420](https://dms.mpeg.expert/doc_end_user/documents/155_Geneva/wg11/MDS26500_WG05_N00420-v2.zip)] and run V-PCC CEs.

## Dependencies 

### Setting-up Python

Use Python 3.12 or newer with the [required packages](requirements.txt). It is advised to set-up a virtual environment. For example:

```shell
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install --upgrade pip
pip install -r requirements.txt
```

### Building MPEG GSC Metrics

Clone the repository, whereby `TAG` is the version specified in the CTC document:

```bash
git clone https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-metrics -b TAG
```

Build mpeg-gsc-metrics based on the [README.md](https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-metrics/-/blob/main/README.md?ref_type=heads) of that project.

### Building MPEG GSC Tools

Clone the repository, whereby `TAG` is the version specified in the CTC document:

```bash
git clone https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-tools -b TAG
```

Build GsTools based on the [README.md](https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-tools/-/blob/main/gsTools/README.md?ref_type=heads) of that project.

### Building MPEG GSC Importance

Clone the repository, whereby `TAG` is the version that matches with TMC2.

```bash
git clone https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-importance -b TAG
```

Build mpeg-gsc-importance based on the [README.md](https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-importance/-/blob/main/README.md?ref_type=heads) of that project.

## Building TMC2

Follow the build instructions on the main [README.md](../README.md) of this project.

## Running experiments

Run the encoder and decoder directly.  This makes each test condition explicit, is suitable for both a single frame and a frame sequence, and lets you use ordinary command-line argument overrides without editing a helper script.

The examples below use these locations:

* `bin/PccGs4AppEncoder` and `bin/PccAppDecoder` are the TMC2 binaries produced by the build.
* `cfg/gs4/` contains the GS configuration files.
* Input frames are Gaussian-splatting PLY files.  For sequences, use a `printf`-style path template such as `/data/sequence/plys/frame_%04i.ply`.
* Output directories must already exist.

### Encode

An encoder invocation consists of the V-PCC GS anchor configuration [gs-ctc-anchor.cfg](../cfg/gs4/gs-ctc-anchor.cfg), and a rate-point configuration such as [hm-main10/rate/r04.cfg](../cfg/gs4/hm-main10/rate/r04.cfg). Configuration files are applied from left to right: a later `--config` value overrides an earlier one. Start with the common configuration, then select a rate point.

```bash
bin/PccGs4AppEncoder \
  --uncompressedDataPath=/data/sequence/plys/frame_%04i.ply \
  --configurationFolder=cfg/gs4/ \
  --config=cfg/gs4/gs-ctc-anchor.cfg \
  --config=cfg/gs4/hm-main10/gs-main10-common.cfg \
  --config=cfg/gs4/hm-main10/rate/r04.cfg \
  --startFrameNumber=81 \
  --frameCount=1 \
  --compressedStreamPath=/output/results/1F/manwithfruit_tracked/r04/1F_manwithfruit_tracked_r04.bin
```

`--uncompressedDataPath` accepts a single PLY path or a template containing a frame-number conversion (for example, `%04i`). `--startFrameNumber` is the first number substituted in that template and `--frameCount` is the number of consecutive frames to encode. If no template is used, use the appropriate single-frame values.

The CTC configuration already includes a packing configuration. You can try alternative packing configurations:

* `cfg/gs4/gs-pack-none.cfg` — do not pack maps.
* `cfg/gs4/gs-pack-attributes.cfg` — pack attributes.
* `cfg/gs4/gs-pack-all-in-one.cfg` — pack all maps.

Rate-point files are in `cfg/gs4/<profile>/rate/`; use a corresponding rate point config for the desired condition. You can override a configuration value on the command line after the `--config` arguments. Commonly used overrides are:

* `--keepIntermediateFiles=1 --debugPath=/output/debug` retains intermediate files for diagnosis.
* `--implicitPacking=0|1` and `--temporalPacking=0|1` control packing behavior when a packing mode is selected.
* `--shExplicitMappingFlag=0|1` selects implicit (0:interleaved) or explicit (1) spherical-harmonics mapping.
* `--isPlanar=0|1` selects planar mapping for spherical harmonics; it is relevant only when explicit mapping is enabled.
* `--enableGeometryPrediction=0|1` enables segmented PEPGA-Chain geometry residual prediction (default: 1).
* `--geometryPredictionSegmentVal=256` sets the geometry-prediction segment size (default: 256; must be greater than 0 when prediction is enabled).
* `--GSColmapFolderPath=/data/sequence/colmap_data/frame_0081/sparse/` supplies COLMAP camera data as SEI metadata.
* `--importanceScoresPath=/data/scores/frame_%04i.impScores.txt` provides one importance-score file per frame for adaptive quantization or LoD.
* `--lodSplitRatios=30,70` enables encoder LoD partitioning. The values define the split ratios; use `--lodTargetLodMask` on the decoder to choose which levels to reconstruct.

Other coding switches from the CTC setup, including SH transform, PCA, non-uniform quantization, prediction, scale/opacity transforms, duplicate merging, raw-patch layout, and quaternion representation, can likewise be passed as `--option=value`. Use `bin/PccGs4AppEncoder --help` for the complete, build-specific option list.

### Decode

Decode the generated bitstream with a matching output template:

```bash
bin/PccAppDecoder \
  --compressedStreamPath=/output/results/1F/manwithfruit_tracked/r04/1F_manwithfruit_tracked_r04.bin \
  --reconstructedDataPath=/output/results/1F/manwithfruit_tracked/r04/plys/1F_manwithfruit_tracked_r04_%04i.decoded.ply \
  --startFrameNumber=81 \
  --computeMetrics=0 \
  --computeChecksum=0
```

`--reconstructedDataPath` is a `printf`-style template for the reconstructed PLY files. Add `--keepIntermediateFiles=1 --debugPath=/output/debug` when decoder intermediates are needed. To decode selected LoD levels, pass a nonzero `--lodTargetLodMask` (a bit mask: `1` selects L0, `2` selects L1, `3` selects L0 and L1, and so on). Use `bin/PccAppDecoder --help` for all decoder arguments.

### Supporting tools and objective evaluation

For camera-aware importance scores and metrics, first add the COLMAP camera header to each source PLY using `gstools` from MPEG GSC Tools:

```bash
gstools camera \
  --input=/data/sequence/plys/frame_0081.ply \
  --output=/output/frame_0081.camerapos.ply \
  --image=/data/sequence/colmap_data/frame_0081/sparse/images.bin \
  --camera=/data/sequence/colmap_data/frame_0081/sparse/cameras.bin
```

When using adaptive quantization or LoD without precomputed scores, generate scores for every input frame with `mpeg-gsc-importance`. The V-PCC GS anchor requires setting `--downscaleFactor=2`:
```bash
mpeg-gsc-importance --threads=2 \
  -a /output/frame_0081.camerapos.ply \
  --impScore=/output/scores/frame_0081.impScores.txt \
  --downscaleFactor=2
```

Pass the resulting template to the encoder with `--importanceScoresPath`. Then evaluate each reconstructed frame with MPEG GSC Metrics; give it the camera-annotated source and decoded PLY, along with the reference view resolution:

```bash
mpeg-gsc-metrics -v 1 --startFrame=0 --frameCount=1 --cpu=1 --threads=2 \
  --useCameraPosition=1 --width=3840 --height=2160 \
  --source=/output/frame_0081.camerapos.ply \
  --decode=/output/results/1F/manwithfruit_tracked/r04/plys/1F_manwithfruit_tracked_r04_0081.decoded.ply \
  --output=/output/results/1F/manwithfruit_tracked/r04/metrics/1F_manwithfruit_tracked_r04_0081.decoded.ply.metric
```

#### Result names required for CSV generation

[gsc_parse_vpcc_results.py](gsc_parse_vpcc_results.py) does not discover files by pattern: it constructs their paths from its arguments. Name the files and directories exactly as shown below, or the script cannot be used. For `-l 1F -s manwithfruit_tracked -r r04 -f 81 -n 32`, its results directory must be `/output/results/1F/manwithfruit_tracked/r04`, and the base name is `1F_manwithfruit_tracked_r04`.

```text
/output/results/1F/manwithfruit_tracked/r04/
├── 1F_manwithfruit_tracked_r04.bin
├── 1F_manwithfruit_tracked_r04_enc_bitstream.txt
├── 1F_manwithfruit_tracked_r04.encoder.err
├── 1F_manwithfruit_tracked_r04.decoder.err
├── plys/
│   └── 1F_manwithfruit_tracked_r04_0081.decoded.ply
├── metrics/
│   ├── 1F_manwithfruit_tracked_r04_0081.decoded.ply.metric
│   └── 1F_manwithfruit_tracked_r04_0081.metric.err
└── additional frames follow the same naming scheme

/output/results/1F/manwithfruit_tracked/importance_scores/  # optional
└── 1F_manwithfruit_tracked_0081.importance.err
```

Create the `_enc_bitstream.txt` file by running `parse_v3c_sample_stream` on the `.bin` file. The `.encoder.err`, `.decoder.err`, `.metric.err`, and optional `.importance.err` files must each contain exactly one timing line in this format:

```text
user_time=<seconds> wall_time=<elapsed> max_rss_KB=<kilobytes> exit_status=<status>
```

For example, GNU `time` can generate the required line with `-f 'user_time=%U wall_time=%E max_rss_KB=%M exit_status=%x'`. The parser expects the same decoded PLY and metric filenames for every frame in the requested range.

Generate a CSV row for this rate point with:

```bash
python3 gsc_parse_vpcc_results.py \
  -d /output/results/1F/manwithfruit_tracked/r04 \
  -l 1F -s manwithfruit_tracked -r r04 -f 81 -n 32
```

The CSV can be compared with [the anchor results](anchor_results/VPCC-GS.csv) and used to populate the reporting template.

To create a comparison workbook, use [gsc_csv2excel.py](gsc_csv2excel.py):

```shell
usage: gsc_csv2excel.py [-h] -t TEMPLATE -a ANC_CSV -p PRO_CSV -x XLSM

Generate XLSM file from CSV data.

options:
  -h, --help            show this help message and exit
  -t TEMPLATE, --template TEMPLATE
                        Path to XLSM template file
  -a ANC_CSV, --anc_csv ANC_CSV
                        Path to anchor CSV file
  -p PRO_CSV, --pro_csv PRO_CSV
                        Path to proposed CSV file
  -x XLSM, --xlsm XLSM  Path to output XLSM file
```

### Posetrace videos for informal expert viewing and subjective evaluation

:construction:
