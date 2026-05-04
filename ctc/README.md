A guide to run CEs to support gaussian splats coding in V-PCC [MDS25755_WG07_N01343](https://dms.mpeg.expert/doc_end_user/documents/152_Geneva/wg11/MDS25755_WG07_N01343.zip)

# Dependencies 

## mpeg-gsc-metrics
Clone the repository:

```bash
git clone https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-metrics
```

Commit used: [215afff8d7be723e1204e44ce6d578d8fb1a4d1a](https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-metrics/-/commit/215afff8d7be723e1204e44ce6d578d8fb1a4d1a)


Build the mpeg-gsc-metrics based on [README.md](https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-metrics/-/blob/main/README.md?ref_type=heads)

## mpeg-gsc-tools

Clone the repository:

```bash
git clone https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-tools
```

Commit used: [f6c7dfa313bf4633922db7c39e6b6312961da61f](https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-tools/-/commit/f6c7dfa313bf4633922db7c39e6b6312961da61f)

Build the GsTools based on [README.md](https://git.mpeg.expert/MPEG/Explorations/GSC/gsc-software/mpeg-gsc-tools/-/blob/main/gsTools/README.md?ref_type=heads)

## Python
Install the `openpyxl` Python package (version 3.1.5) to handle XLSM files.

```bash
python3 -m pip install openpyxl==3.1.5
```

## TMC2 PCCGSEncoder_decoder branch 

Clone the repository:

```bash
git clone https://git.mpeg.expert/kondrad/mpeg-pcc-tmc2
git checkout PCCGSEncoder_decoder
```

Build the mpeg-pcc-tmc2 

# Core Experiment 

Edit paths in [CE-gen-scripts.sh](./CE-gen-scripts.sh)

```bash
DATASET_DIR="/path/to/CTC"
METRIC_DIR="/path/to/mpeg-gsc-metrics/build/Release/bin"
GSCTOOL_DIR="/path/to/mpeg-gsc-tools"
CAMERAPOS_DIR="$GSCTOOL_DIR/gsTools/build/msvc/Release/bin"
TMC_DIR="/path/to/vmpeg-pcc-tmc2"
BASE_OUTPUT="/path/to/CE"

```

run [CE-gen-scripts.sh](https://git.mpeg.expert/MPEG/3dgh/v-pcc/software/mpeg-pcc-tmc2/-/blob/Amd1_main_draft/gsc_scripts/CE-gen-scripts.sh?ref_type=heads) to run the CE.  

```bash
bash CE-gen-scripts.sh
```

The scripts creates the following scripts in the `output/jobs` directory:

* `run_all.sh` – Run all jobs in parallel locally
* `run_all_slurm.sh` – Run all jobs in parallel on Slurm
* `step1_run_codec.sh` – VPCC encode/decode jobs
* `step2_run_metric.sh` – Metric calculation jobs
* `step3_run_summary.sh` – Summary results

The scripts will run automatically if `AUTO-RUN` is set to 1 in [CE-gen-scripts.sh](./CE-gen-scripts.sh).
