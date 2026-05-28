# Conducting V-PCC GS experiments

## Introduction

This is a guide to generate the V-PCC anchor [[WG 4 N 826](https://dms.mpeg.expert/doc_end_user/documents/154_SantaEularia/wg11/MDS26322_WG04_N00826-v3.zip)] and run V-PCC CEs.

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

> [!NOTE]
> The mpeg-gsc-metrics submodule is a special version of mpeg-gsc-metrics that is used for importance score calcuation. This submodule is built automatically as part of the TMC2 build.

### Building MPEG GSC Tools

Clone the repository, whereby `TAG` is the version specified in the CTC document:

```bash
git clone https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-tools -b TAG
```

Build GsTools based on the [README.md](https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-tools/-/blob/main/gsTools/README.md?ref_type=heads) of that project.

## Building TMC2

Follow the build instructions on the main [README.md](../README.md) of this project.

## Running experiments

The script [CE-gen-scripts.sh](CE-gen-scripts.sh) is used to prepare and run experiments. At the top of the script there are three sections with parameters:

* Experiment settings
* Coding settings
* Base directories

The most commonly changed parameters such as the base directories and the experiment mode (which sequences to run), can be set using environment variables. Other parameters require you to edit the script. For example, to run the anchor for mandatory sequences (EXPERIMENT_MODE= 0), with two threads per metrics job, run:

```bash
export EXPERIMENT_MODE=2
export METRICS_THREADS=2
export DATASET_DIR=/my/data/ctc
export METRIC_DIR=/my/software/mpeg-gsc-metrics/build/Release/bin
export GSCTOOL_DIR=/my/software/mpeg-gsc-tools
export CAMERAPOS_DIR=/my/software/gsTools/build
export TMC_DIR=/my/software/mpeg-pcc-tmc2
export BASE_OUTPUT=/my/data/out
export PYTHON=/usr/bin/python3.12
bash ctc/CE-gen.scripts.sh
```

The script generates a directory structure in a sub-directory of the `BASE_OUTPUT` directory, with a name that depends on experiment settings and the current time, e.g. `V-PCC-GS-pack-attr-26-05-28-153742/`.

### Launching the experiment

Within that directory, there will be a subdirectory `jobs/` in which the following scripts will be generated:

* `run_all.sh` – Run all jobs in parallel locally
* `run_all_slurm.sh` – Run all jobs in parallel on Slurm
* `step0_run_annotate.sh` - Importance score jobs
* `step1_run_codec.sh` – VPCC encode/decode jobs
* `step2_run_metric.sh` – Metric calculation jobs
* `step3_run_summary.sh` – Summary results

The scripts will run automatically if `AUTO-RUN` is set to 1 in [CE-gen-scripts.sh](./CE-gen-scripts.sh).

### Objective evaluation

The objective results will be in the `metrics/` sub-directory next to the `jobs/` directory. Two files are generated:

* a CSV file in the same format as [the anchor results](anchor_results/VPCC-GS.csv)
* Filled-in reporting template

To compare against another reference, use [gsc_csv2excel.py](gsc_csv2excel.py):

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
