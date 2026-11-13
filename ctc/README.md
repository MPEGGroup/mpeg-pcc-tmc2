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

## Computing the V-PCC GS anchor

The commands below run the encoder, decoder, and objective metrics directly. It is advised to create your own run script to run tasks in parallel. Run the commands from the TMC2 repository root, where `bin/` and `cfg/` are located.

* `bin/PccGs4AppEncoder` and `bin/PccAppDecoder` are the TMC2 binaries produced by the build.
* `cfg/gs4/` contains the GS configuration files.
* Each CTC sequence has Gaussian-splatting PLY files under `plys/` and COLMAP camera files under `colmap_data/`. The commands below show the `manwithfruit_tracked` sequence; use the sequence and frame range specified by the CTC content.
* Create the output directories before running the commands.

The anchor test set has two lengths: `1F` encodes one frame, and `NF` encodes 32 consecutive frames. For each applicable sequence and length, encode the four `C1` rate points (`r01`–`r04`) and the `CW` near-lossless condition (`lossless`). Rate configurations for the HM Main 10 profile are under `cfg/gs4/hm-main10/rate/`. The `lossless.cfg` is a best-effort near-lossless configuration, not a guarantee of mathematically lossless reconstruction.

The example below is the `1F`, `r04` condition for `manwithfruit_tracked`, whose first frame is 81. For an `NF` run, use `--frameCount=32`; use the relevant first-frame number for each sequence.

### Encode

Create the result and frame-processing directories first. For the HM Main 10 CTC conditions, apply [gs-ctc-anchor.cfg](../cfg/gs4/gs-ctc-anchor.cfg), then the corresponding rate-point file such as [r04.cfg](../cfg/gs4/hm-main10/rate/r04.cfg).

```bash
mkdir -p /output/results/1F/manwithfruit_tracked/r04/plys \
  /output/results/1F/manwithfruit_tracked/r04/metrics \
  /output/results/frames/manwithfruit_tracked/frames \
  /output/results/1F/manwithfruit_tracked/importance_scores

bin/PccGs4AppEncoder \
  --uncompressedDataPath=/data/sequence/plys/frame_%04i.ply \
  --configurationFolder=cfg/gs4/ \
  --config=cfg/gs4/gs-ctc-anchor.cfg \
  --config=cfg/gs4/hm-main10/rate/r04.cfg \
  --importanceScoresPath=/output/results/frames/manwithfruit_tracked/frames/frame_%04i.impScores.txt \
  --startFrameNumber=81 \
  --frameCount=1 \
  --compressedStreamPath=/output/results/1F/manwithfruit_tracked/r04/1F_manwithfruit_tracked_r04.bin
```

`--uncompressedDataPath` accepts a single PLY path or a template containing a frame-number conversion (for example, `%04i`). `--startFrameNumber` is the first number substituted in that template and `--frameCount` is the number of consecutive frames to encode. The importance-score template must resolve to one score file for each encoded frame; see [Supporting tools and objective evaluation](#supporting-tools-and-objective-evaluation).

### Non-CTC/custom coding options

Do not add coding overrides when generating the anchor. In particular, keep the packing, SH transform, quantization, and geometry-prediction settings in the CTC configuration files. The options below are examples for separate experiments, not part of the anchor:

* `cfg/gs4/gs-pack-none.cfg` — do not pack maps.
* `cfg/gs4/gs-pack-attributes.cfg` — pack attributes.
* `cfg/gs4/gs-pack-all-in-one.cfg` — pack all maps.

For a custom run, a configuration value can be overridden on the command line after the `--config` arguments. Examples include:

* `--keepIntermediateFiles=1 --debugPath=/output/debug` retains intermediate files for diagnosis.
* `--implicitPacking=0|1` and `--temporalPacking=0|1` control packing behavior when a packing mode is selected.
* `--shMappingMode=VALUE` selects a spherical-harmonics mapping mode; use the encoder help and relevant configuration files for supported values.
* `--enableGeometryPrediction=0|1` and `--geometryPredictionSegmentVal=VALUE` control segmented PEPGA-Chain geometry residual prediction.
* `--GSColmapFolderPath=/data/sequence/colmap_data/frame_0081/sparse/` supplies COLMAP camera data as SEI metadata.
* `--lodSplitRatios=30,70` enables encoder LoD partitioning. The values define the split ratios; use `--lodTargetLodMask` on the decoder to choose which levels to reconstruct.

Use `bin/PccGs4AppEncoder --help` and `bin/PccAppDecoder --help` for the complete, build-specific option lists.

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

The V-PCC GS anchor uses importance scores. For every source frame to be encoded, first add the COLMAP camera header to its PLY using `gstools` from MPEG GSC Tools:

```bash
gstools camera \
  --input=/data/sequence/plys/frame_0081.ply \
  --output=/output/results/frames/manwithfruit_tracked/frames/frame_0081.camerpos.ply \
  --image=/data/sequence/colmap_data/frame_0081/sparse/images.bin \
  --camera=/data/sequence/colmap_data/frame_0081/sparse/cameras.bin
```

Then calculate an importance-score file for each frame with `mpeg-gsc-importance`. Use `--downscaleFactor=2` for the V-PCC GS anchor. It is advised to generate scores once per source frame and reuse them across rate points:

```bash
mpeg-gsc-importance --threads=2 \
  -a /output/results/frames/manwithfruit_tracked/frames/frame_0081.camerpos.ply \
  --impScore=/output/results/frames/manwithfruit_tracked/frames/frame_0081.impScores.txt \
  --downscaleFactor=2
```

Pass the resulting template to the encoder with `--importanceScoresPath`. Then evaluate each reconstructed frame with MPEG GSC Metrics; give it the camera-annotated source and decoded PLY, along with the reference view resolution:

```bash
mpeg-gsc-metrics --verbose=1 --save=0 \
  --startFrame=0 --frameCount=1 --cpu=1 --threads=2 \
  --useCameraPosition=1 --width=3840 --height=2160 \
  --source=/output/results/frames/manwithfruit_tracked/frames/frame_0081.camerpos.ply \
  --decode=/output/results/1F/manwithfruit_tracked/r04/plys/1F_manwithfruit_tracked_r04_0081.decoded.ply \
  --output=/output/results/1F/manwithfruit_tracked/r04/metrics/1F_manwithfruit_tracked_r04_0081.decoded.ply.metric
```

Run the metrics command once per decoded frame. Use the reference view width and height for the selected sequence; the example values above are for `manwithfruit_tracked`.

#### Result names required for CSV generation

[gsc_parse_vpcc_results.py](gsc_parse_vpcc_results.py) constructs paths from its arguments; it does not search for result files. For the `1F`, `manwithfruit_tracked`, `r04` example, use `-f 81 -n 1` (one frame). For an `NF` result, use `-n 32`. The results directory is `/output/results/1F/manwithfruit_tracked/r04`, and the base name is `1F_manwithfruit_tracked_r04`.

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

The `.encoder.err`, `.decoder.err`, each `.metric.err`, and each `.importance.err` file must contain exactly one timing line in this format:

```text
user_time=<seconds> wall_time=<elapsed> max_rss_KB=<kilobytes> exit_status=<status>
```

GNU `time` can generate the line with `-f 'user_time=%U wall_time=%E max_rss_KB=%M exit_status=%x'`. Write timing output to the matching `.err` file and keep any full command log separately. The parser expects a decoded PLY and metric file for every frame in the requested range.

Initialize the proposal CSV with its header, then append one row for each completed sequence, length, and rate point. For the example condition:

```bash
python3 ctc/gsc_parse_vpcc_results.py --print-header-row > /output/VPCC-GS.csv
python3 ctc/gsc_parse_vpcc_results.py \
  -d /output/results/1F/manwithfruit_tracked/r04 \
  -l 1F -s manwithfruit_tracked -r r04 -f 81 -n 1 \
  >> /output/VPCC-GS.csv
```

The CSV can be compared with [the anchor results](anchor_results/VPCC-GS.csv) and used to populate the reporting template.

To create a comparison workbook, use [gsc_csv2excel.py](gsc_csv2excel.py):

```bash
python3 ctc/gsc_csv2excel.py \
  -t ctc/templates/WG5N420_template.xlsm \
  -a ctc/anchor_results/VPCC-GS.csv \
  -p /output/VPCC-GS.csv \
  -x /output/WG5N420_results.xlsm
```

### Posetrace videos for informal expert viewing and subjective evaluation

:construction:
