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

#!/bin/bash
set -euo pipefail

declare -A ENABLE_SEQs
declare -A PLY_DIRs
declare -A COLMAP_DIRs
declare -A RESOLUTIONS

###########################################################################################
## Experiment settings
AUTO_RUN=0                 # 0:off, 1:run_local, 2:run_slurm 
FLAG_1F=1                  # 0 or 1
FRAME_COUNT=1              # Number of frames to encode (1 for 1F)
GENERATE_PNGs=0            # 0 or 1
MET_FCOUNT=32 # or 999: ~ last 
COLMAP_CONVERT_TO_TEXT=0   #set it to 1 if you only have bin files (typically you need to run this only once)
MAPMODE=1 # 0: INTERLEAVED, 1: PLANAR(CTC)
EXPERIMENT_MODE=0 # 0: mandatory, 1: all, 2: man_with_fruit_tracked only
PACKING_MODE=1 # 0: pack-none, 1: pack-attr, 2: pack-all
SH_MAPPING=1 # set to 0 to use interleaved maping in the video bitstream (1 corresponds to CTC, i.e. planar mapping)
COLMAP_SEI=0   #set it to 1 if you want to send the camera pareters as an SEI message

# Gaussian Splatting Flags
GSD_TRANSFORM_2D=1     #1: enable 2D transform for sphereical harmnoics, 0: disable
GSD_ADAPTIVE_SH=1      #  when GSD_TRANSFORM_2D=1, 1: enable m75790, 0: enable m75380
GSD_NON_UNIFORM_QUANT=1 # 1: enable m75025, 0: disable m75025

JOBNAME="V-PCC-GS4-mandatory"

# ===============================
# Base Directory - CHANGE THIS FOR YOUR SYSTEM
# ===============================
DATASET_DIR="/path/to/CTC"
METRIC_DIR="/path/to/mpeg-gsc-metrics/build/Release/bin"
GSCTOOL_DIR="/path/to/mpeg-gsc-tools"
CAMERAPOS_DIR="$GSCTOOL_DIR/gsTools/build"
TMC_DIR="/path/to/vmpeg-pcc-tmc2"
BASE_OUTPUT="/path/to/CE"
TIME="/bin/time "

SCRIPT_PATH="$(readlink -f "$0" 2>/dev/null || realpath "$0")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"

# ===============================
# Dataset Directories (same order as TRACKS)
# ===============================
DIRS=(
 "bartender_tracked"
 "bartender_semitracked"
 "cinema_tracked"
 "cinema_semitracked"
 "breakfast_tracked"
 "breakfast_semitracked"
 "breakfast_untracked"
 "breakdance_untracked"
 "trio"
 "makeup"
 "musical"
 "manwithfruit_tracked"
 "lego_ferrari"
 "lego_bugatti"
 "cricket_player"
 "plant"
 "solo_tango_female"
 "solo_tango_male"
 "tango_duo"
 "tennis_player"
 "library"
 "flowerdance"
 "gymnast"
)

# ===================================
# Enable sequences for the experiment
# ===================================

ENABLE_SEQs[bartender_tracked]=1
ENABLE_SEQs[bartender_semitracked]=1
ENABLE_SEQs[cinema_tracked]=1
ENABLE_SEQs[cinema_semitracked]=1
ENABLE_SEQs[breakfast_tracked]=1
ENABLE_SEQs[breakfast_semitracked]=1
ENABLE_SEQs[breakfast_untracked]=1
ENABLE_SEQs[breakdance_untracked]=0
ENABLE_SEQs[trio]=0
ENABLE_SEQs[makeup]=0
ENABLE_SEQs[musical]=0
ENABLE_SEQs[manwithfruit_tracked]=1
ENABLE_SEQs[lego_ferrari]=1
ENABLE_SEQs[lego_bugatti]=1
ENABLE_SEQs[cricket_player]=1
ENABLE_SEQs[plant]=1
ENABLE_SEQs[solo_tango_female]=1
ENABLE_SEQs[solo_tango_male]=1
ENABLE_SEQs[tango_duo]=1
ENABLE_SEQs[tennis_player]=1
ENABLE_SEQs[library]=0
ENABLE_SEQs[flowerdance]=1
ENABLE_SEQs[gymnast]=1

if [ $EXPERIMENT_MODE -eq 1 ]; then
    for SEQ in "${!ENABLE_SEQs[@]}"; do
        ENABLE_SEQs[$SEQ]=1
    done
    JOBNAME="V-PCC-GS4-all"
elif [ $EXPERIMENT_MODE -eq 2 ]; then
    for SEQ in "${!ENABLE_SEQs[@]}"; do
        ENABLE_SEQs[$SEQ]=0
    done
    ENABLE_SEQs[manwithfruit_tracked]=1
    JOBNAME="V-PCC-GS4-manwithfruit"
fi

# ===============================
# DIRs for Point Clouds
# ===============================
PLY_DIRs[bartender_tracked]="$DATASET_DIR/${DIRS[0]}/plys"
PLY_DIRs[bartender_semitracked]="$DATASET_DIR/${DIRS[1]}/plys"
PLY_DIRs[cinema_tracked]="$DATASET_DIR/${DIRS[2]}/plys"
PLY_DIRs[cinema_semitracked]="$DATASET_DIR/${DIRS[3]}/plys"
PLY_DIRs[breakfast_tracked]="$DATASET_DIR/${DIRS[4]}/plys"
PLY_DIRs[breakfast_semitracked]="$DATASET_DIR/${DIRS[5]}/plys"
PLY_DIRs[breakfast_untracked]="$DATASET_DIR/${DIRS[6]}/plys"
PLY_DIRs[breakdance_untracked]="$DATASET_DIR/${DIRS[7]}/plys"
PLY_DIRs[trio]="$DATASET_DIR/${DIRS[8]}/plys"
PLY_DIRs[makeup]="$DATASET_DIR/${DIRS[9]}/plys"
PLY_DIRs[musical]="$DATASET_DIR/${DIRS[10]}/plys"
PLY_DIRs[manwithfruit_tracked]="$DATASET_DIR/${DIRS[11]}/plys"
PLY_DIRs[lego_ferrari]="$DATASET_DIR/${DIRS[12]}/plys"
PLY_DIRs[lego_bugatti]="$DATASET_DIR/${DIRS[13]}/plys"
PLY_DIRs[cricket_player]="$DATASET_DIR/${DIRS[14]}/plys"
PLY_DIRs[plant]="$DATASET_DIR/${DIRS[15]}/plys"
PLY_DIRs[solo_tango_female]="$DATASET_DIR/${DIRS[16]}/plys"
PLY_DIRs[solo_tango_male]="$DATASET_DIR/${DIRS[17]}/plys"
PLY_DIRs[tango_duo]="$DATASET_DIR/${DIRS[18]}/plys"
PLY_DIRs[tennis_player]="$DATASET_DIR/${DIRS[19]}/plys"
PLY_DIRs[library]="$DATASET_DIR/${DIRS[20]}/plys"
PLY_DIRs[flowerdance]="$DATASET_DIR/${DIRS[21]}/plys"
PLY_DIRs[gymnast]="$DATASET_DIR/${DIRS[22]}/plys"
 
# ===============================
# DIRs for COLMAP Data
# ===============================
COLMAP_DIRs[bartender_tracked]="$DATASET_DIR/${DIRS[0]}/colmap_data"
COLMAP_DIRs[bartender_semitracked]="$DATASET_DIR/${DIRS[1]}/colmap_data"
COLMAP_DIRs[cinema_tracked]="$DATASET_DIR/${DIRS[2]}/colmap_data"
COLMAP_DIRs[cinema_semitracked]="$DATASET_DIR/${DIRS[3]}/colmap_data"
COLMAP_DIRs[breakfast_tracked]="$DATASET_DIR/${DIRS[4]}/colmap_data"
COLMAP_DIRs[breakfast_semitracked]="$DATASET_DIR/${DIRS[5]}/colmap_data"
COLMAP_DIRs[breakfast_untracked]="$DATASET_DIR/${DIRS[6]}/colmap_data"
COLMAP_DIRs[breakdance_untracked]="$DATASET_DIR/${DIRS[7]}/colmap_data"
COLMAP_DIRs[trio]="$DATASET_DIR/${DIRS[8]}/colmap_data"
COLMAP_DIRs[makeup]="$DATASET_DIR/${DIRS[9]}/colmap_data"
COLMAP_DIRs[musical]="$DATASET_DIR/${DIRS[10]}/colmap_data"
COLMAP_DIRs[manwithfruit_tracked]="$DATASET_DIR/${DIRS[11]}/colmap_data"
COLMAP_DIRs[lego_ferrari]="$DATASET_DIR/${DIRS[12]}/colmap_data"
COLMAP_DIRs[lego_bugatti]="$DATASET_DIR/${DIRS[13]}/colmap_data"
COLMAP_DIRs[cricket_player]="$DATASET_DIR/${DIRS[14]}/colmap_data"
COLMAP_DIRs[plant]="$DATASET_DIR/${DIRS[15]}/colmap_data"
COLMAP_DIRs[solo_tango_female]="$DATASET_DIR/${DIRS[16]}/colmap_data"
COLMAP_DIRs[solo_tango_male]="$DATASET_DIR/${DIRS[17]}/colmap_data"
COLMAP_DIRs[tango_duo]="$DATASET_DIR/${DIRS[18]}/colmap_data"
COLMAP_DIRs[tennis_player]="$DATASET_DIR/${DIRS[19]}/colmap_data"
COLMAP_DIRs[library]="$DATASET_DIR/${DIRS[20]}/colmap_data"
COLMAP_DIRs[flowerdance]="$DATASET_DIR/${DIRS[21]}/colmap_data"
COLMAP_DIRs[gymnast]="$DATASET_DIR/${DIRS[22]}/colmap_data"

# ===============================
# RESOLUTIONS
# ===============================
RESOLUTIONS[bartender_tracked]="1920x1080"
RESOLUTIONS[bartender_semitracked]="1920x1080"
RESOLUTIONS[cinema_tracked]="1920x1080"
RESOLUTIONS[cinema_semitracked]="1920x1080"
RESOLUTIONS[breakfast_tracked]="1920x1080"
RESOLUTIONS[breakfast_semitracked]="1920x1080"
RESOLUTIONS[breakfast_untracked]="1920x1080"
RESOLUTIONS[breakdance_untracked]="1920x1080"
RESOLUTIONS[trio]="1920x1080"
RESOLUTIONS[makeup]="1920x1080"
RESOLUTIONS[musical]="1920x1080"
RESOLUTIONS[manwithfruit_tracked]="3840x2160"
RESOLUTIONS[lego_ferrari]="4594x5514"
RESOLUTIONS[lego_bugatti]="3852x2868"
RESOLUTIONS[cricket_player]="4520x2540"
RESOLUTIONS[plant]="2954x3968"
RESOLUTIONS[solo_tango_female]="4534x2542"
RESOLUTIONS[solo_tango_male]="4528x2544"
RESOLUTIONS[tango_duo]="4522x2538"
RESOLUTIONS[tennis_player]="4518x2540"
RESOLUTIONS[library]="3793x2131"
RESOLUTIONS[flowerdance]="2456x2054"
RESOLUTIONS[gymnast]="2456x2054"

###########################################################################################
PYTHON="python3"
METRIC="$METRIC_DIR/mpeg-gsc-metrics"
CAMERAPOS_BIN="$CAMERAPOS_DIR/cameraPosition"
CFG_PATH="$TMC_DIR/cfg/gs4/"
TMC_ENC_BIN="$TMC_DIR/bin/PccGs4AppEncoder"
TMC_DEC_BIN="$TMC_DIR/bin/PccAppDecoder"
PARSER_SCRIPT="$SCRIPT_DIR/gsc_parse_vpcc_results.py"
EXTRACT_IMAGES_SCRIPT="$SCRIPT_DIR/extract_images.py"

PACK_CONFIG=$CFG_PATH/gs-pack-none.cfg
if [ $PACKING_MODE -eq 0 ]; then
    JOBNAME="$JOBNAME-pack-none"
    PACK_CONFIG=$CFG_PATH/gs-pack-none.cfg
elif [ $PACKING_MODE -eq 1 ]; then
    JOBNAME="$JOBNAME-pack-attr"
    PACK_CONFIG=$CFG_PATH/gs-pack-attributes.cfg
elif [ $PACKING_MODE -eq 2 ]; then
    JOBNAME="$JOBNAME-pack-all"
    PACK_CONFIG=$CFG_PATH/gs-pack-all-in-one.cfg
fi 

###########################################################################################

# Validate dataset PLY_DIRs
for SEQ in "${!PLY_DIRs[@]}"; do
    if [ "${ENABLE_SEQs[$SEQ]:-0}" -ne 1 ]; then
        echo "⏭️ Skipping $SEQ (disabled)"
        continue
    fi
    PLY_PATH="${PLY_DIRs[$SEQ]}"
    if [[ ! -d "$PLY_PATH" ]]; then
        echo "❌ $PLY_PATH does not exist"
        #exit 1
    else
      echo "✅ $PLY_PATH  will be processed"
    fi
    shopt -s nullglob
    files=()
    for f in "$PLY_PATH"/*.ply; do
        [[ "$f" != *.camerapos.ply ]] && files+=("$f")
    done
    if (( ${#files[@]} < 1 )); then
        echo "❌ No *.ply in $PLY_PATH (w/o .camerapos.ply)"
        #exit 1
    else
        echo "✅ found ${#files[@]} plys for processing"
    fi
done
#convert bin colmap files to txt
if [ $COLMAP_CONVERT_TO_TEXT == 1 ]; then
    for SEQ in "${!COLMAP_DIRs[@]}"; do
        if [ "${ENABLE_SEQs[$SEQ]:-0}" -ne 1 ]; then
            echo "⏭️ Skipping $SEQ (disabled)"
            continue
        fi
        COLMAP_DIR="${COLMAP_DIRs[$SEQ]}"
        if [ -d "$COLMAP_DIR" ]; then
            echo "Processing COLMAP data $COLMAP_DIR"
            colmap model_converter \
                --input_path "$COLMAP_DIR/frame_0000/sparse" \
                --output_path "$COLMAP_DIR/frame_0000/sparse" \
                --output_type TXT
        fi
    done
fi

WRITE() { echo "$2" >> "$1"; }
MKDIRCK() { [[ -d "$1" ]] || mkdir -p "$1"; }


CURTIME="$(date "+%y-%m-%d-%H%M%S")"
OUT="$BASE_OUTPUT/$JOBNAME-$CURTIME"
BATCH_CODEC="$OUT/jobs/batch_codec"
BATCH_METRIC="$OUT/jobs/batch_metric"
LOG_DIR="$OUT/jobs/logs"

MKDIRCK "$OUT/metrics"
MKDIRCK "$BATCH_CODEC"
MKDIRCK "$BATCH_METRIC"
MKDIRCK "$OUT/bin"
MKDIRCK "$LOG_DIR"
MKDIRCK "$OUT/jobs/done/codec"
MKDIRCK "$OUT/jobs/done/metric"

cp "$TMC_ENC_BIN" "$OUT/bin/$(basename $TMC_ENC_BIN)"
if [ $TMC_ENC_BIN != $TMC_DEC_BIN ]; then
    cp "$TMC_DEC_BIN" "$OUT/bin/$(basename $TMC_DEC_BIN)"
fi

TMC_ENC_BIN="$OUT/bin/$(basename $TMC_ENC_BIN)"
TMC_DEC_BIN="$OUT/bin/$(basename $TMC_DEC_BIN)"

cp "$CAMERAPOS_BIN" "$OUT/bin/$(basename $CAMERAPOS_BIN)"
CAMERAPOS_BIN="$OUT/bin/$(basename $CAMERAPOS_BIN)"

cd "$OUT/jobs"
RUN_CODEC="$OUT/jobs/step1_run_codec.sh"
RUN_METRIC="$OUT/jobs/step2_run_metric.sh"
RUN_SUMMARY="$OUT/jobs/step3_run_summary.sh"

# Initialize run scripts
: > "$RUN_CODEC"
: > "$RUN_METRIC"
: > "$RUN_SUMMARY"

if [ $FLAG_1F == 1 ]; then
    csv_file="$OUT/metrics/VPCC-1F-$JOBNAME.csv"
else
    csv_file="$OUT/metrics/VPCC-$JOBNAME.csv"
fi

echo "seq, rate, total_Byte, pos_Byte, sh0_Byte, sh1_Byte, sh2_Byte, sh3_Byte, rot_Byte, scale_Byte, opacity_Byte, meta_Byte,total_encT, pos_encT, att_encT, total_decT, pos_decT, att_decT, enc_MaxRSS, dec_MaxRSS, RGB-PSNR, YUV-PSNR, YUV-SSIM, YUV-IVSSIM, RGB-LPIPS, GT-RGB-PSNR, renT" > "$csv_file"

for SEQ in "${!PLY_DIRs[@]}"; do
    if [ "${ENABLE_SEQs[$SEQ]:-0}" -ne 1 ]; then
        echo "⏭️ Skipping $SEQ (disabled)"
        continue
    fi
    echo "Processing sequence $SEQ"
    PLY_PATH="${PLY_DIRs[$SEQ]}"
    if [[ ! -d "$PLY_PATH" ]]; then
        echo "❌ $PLY_PATH does not exist"
        continue
    else
      echo "✅ $PLY_PATH  will be processed"
    fi
    COLMAP_DIR="${COLMAP_DIRs[$SEQ]}"
    IFS="x" read -r REN_W REN_H <<< "${RESOLUTIONS[$SEQ]}"
    files=()
    for f in "$PLY_PATH"/*.ply; do
        [[ "$f" != *.camerapos.ply ]] && files+=("$f")
    done
    if [ $FLAG_1F == 1 ]; then
        SelFILEs=("${files[0]}")
    else
        SelFILEs=("${files[@]}")
    fi
    echo "${CFG_PATH}"
    CFG_RATE_DIRs=($CFG_PATH/rate/*)
    for CFG_RATE_DIR in "${CFG_RATE_DIRs[@]}"; do
        RATE="$(basename "$CFG_RATE_DIR")"
        RESULT_DIR="$OUT/results/$SEQ/$RATE"
        MKDIRCK "$RESULT_DIR"

        echo "$SEQ/$RATE"
        count=0
        met_fcount=$MET_FCOUNT
        for FILE in "${SelFILEs[@]}"; do
            FRAME="$(basename "$FILE" .ply)"
            FRAME_NR="${FRAME##*[^0-9]}"
            # Encoding
            JOBFILE="$BATCH_CODEC/$JOBNAME-$CURTIME-$SEQ-$RATE-$FRAME.sh"
            {
                echo "#!/bin/bash"
                echo "#SBATCH --output=$OUT/jobs/logs/%j.out"
                echo "#SBATCH --ntasks-per-node=1"
                echo "#SBATCH --nodes=1"
                echo "#SBATCH --mem-per-cpu=8G"
                echo ""
                echo "# Printing the md5sum of the input PLY for debugging"
                echo "md5sum $FILE"
                echo ""
                echo ""
                echo "# Encode"
                echo "$TIME$TMC_ENC_BIN \\"
                echo "    --uncompressedDataPath=$FILE \\"
                echo "    --configurationFolder=$CFG_PATH/ \\"
                echo "    --config=$CFG_PATH/gs-scc-common.cfg \\"
                echo "    --config=$PACK_CONFIG \\"
                echo "    --config=$CFG_RATE_DIR/encoder.cfg \\"
                echo "    --startFrameNumber=$FRAME_NR \\"
                echo "    --frameCount=1 \\"
				if (( $SH_MAPPING == 0 )); then
				  echo "    --shMappingMode=0 \\"
				fi
				if (( $COLMAP_SEI == 1 )); then
				  echo "    --GSColmapFolderPath=\"$COLMAP_DIR/frame_${FRAME_NR: -4}/sparse/\" \\"
				fi
                echo "    --gsdTransform2dPresentFlag=$GSD_TRANSFORM_2D \\"
                echo "    --dataAdaptiveShTransformEnabled=$GSD_ADAPTIVE_SH \\"
                echo "    --enableShNonUniformQuantization=$GSD_NON_UNIFORM_QUANT \\"
                echo "    --compressedStreamPath=$RESULT_DIR/$FRAME.bin \\"
                echo "    2>&1 | tee $RESULT_DIR/$FRAME.encoder.log \\"
                echo "# Decode"
                echo "$TIME$TMC_DEC_BIN \\"
                echo "    --compressedStreamPath=$RESULT_DIR/$FRAME.bin \\"
                echo "    --reconstructedDataPath=$RESULT_DIR/$FRAME.decoded.ply \\"
                echo "    --computeMetrics=0 --computeChecksum=0 \\"
                echo "    2>&1 | tee $RESULT_DIR/$FRAME.decoder.log \\"
                echo ""
                echo "touch $OUT/jobs/done/codec/\$SLURM_JOB_ID"
            } > "$JOBFILE"
            chmod +x "$JOBFILE"
            echo "$JOBFILE > $LOG_DIR/$JOBNAME-$CURTIME-$SEQ-$RATE-$FRAME.codec.log " >> "$RUN_CODEC"
            if (( count >= met_fcount )); then
                continue
            fi

            # Metrics
            JOBFILE_MET="$BATCH_METRIC/$JOBNAME-$CURTIME-$SEQ-$RATE-$FRAME.sh"
            {
                echo "#!/bin/bash"
                echo "#SBATCH --output=$OUT/jobs/logs/%j.out"
                echo "#SBATCH --ntasks-per-node=1"
                echo "#SBATCH --nodes=1"
                
                # We measured a peak memory consumption of 45.1 GB [m76044] < 7 x 8G = 56G
                echo "#SBATCH --mem-per-cpu=8G"
                echo "#SBATCH --cpus-per-task=7"

                echo "# Add camera header to source"
                echo "$CAMERAPOS_BIN \\"
                echo "    --input=\"$FILE\" \\"
                echo "    --output=\"$RESULT_DIR/$FRAME.camerpos.ply\" \\"
                echo "    --image=\"$COLMAP_DIR/frame_${FRAME_NR: -4}/sparse/images.bin\" \\"
                echo "    --camera=\"$COLMAP_DIR/frame_${FRAME_NR: -4}/sparse/cameras.bin\""
                echo "# Calculate all metrics"
                echo "$TIME$METRIC -v 1 --save=1 --startFrame=0 --frameCount=1 --cpu=1 --useCameraPosition=1 --threads=7 \\"
                echo "    --width=$REN_W --height=$REN_H \\"
                echo "    --source=\"$RESULT_DIR/$FRAME.camerpos.ply\" \\"
                echo "    --decode=\"$RESULT_DIR/$FRAME.decoded.ply\" \\"
                echo "    --output=\"$RESULT_DIR/$FRAME.decoded.ply.metric\""
                if [ $GENERATE_PNGs == 1 ]; then
                    echo "# Convert to PNG images"
                    echo "RGB_PATH=\$(find \"$RESULT_DIR\" -type f -name \"*_img_dec_*\" | head -n1 || true)"
                    echo 'if [[ -n "${RGB_PATH:-}" ]]; then'
                    echo '    if [[ "$RGB_PATH" == *p444* ]]; then PLANAR=1'
                    echo '    elif [[ "$RGB_PATH" == *i444* ]]; then PLANAR=0'
                    echo '    else'
                    echo '        echo "Could not detect planar/interleaved from: $RGB_PATH"'
                    echo '        echo "Default: PLANAR=$PLANAR"'
                    echo '    fi'
                    echo "    $PYTHON $SCRIPT_DIR/$EXTRACT_IMAGES_SCRIPT --input=\"\$RGB_PATH\" --output="$RESULT_DIR" --width=$REN_W  --height=$REN_H --planar=\$PLANAR"
                    echo 'fi'
                fi
                echo "touch $OUT/jobs/done/metric/\$SLURM_JOB_ID"
            } > "$JOBFILE_MET"
            chmod +x "$JOBFILE_MET"
            echo "$JOBFILE_MET > $LOG_DIR/$JOBNAME-$CURTIME-$SEQ-$RATE-$FRAME.metric.log" >> "$RUN_METRIC"
            count=$((count+1))
        done

        echo "$PYTHON $PARSER_SCRIPT -d \"$RESULT_DIR\" -s \"$SEQ\" -r \"$RATE\" -f $FLAG_1F >> \"$csv_file\"" >> "$RUN_SUMMARY"
    done
done

if [ $FLAG_1F == 1 ]; then
    echo "$PYTHON $SCRIPT_DIR/gsc_csv2excel.py -f 1 -t \"$SCRIPT_DIR/templates/WG07N1414_template.xlsm\" -a \"$SCRIPT_DIR/anchor_results/VPCC-1F.csv\" -p \"$csv_file\" -x \"$OUT/metrics/VPCC-1F-Anchor__vs__proposed.xlsm\"" >> "$RUN_SUMMARY"
else
    echo "$PYTHON $SCRIPT_DIR/gsc_csv2excel.py -f 0 -t \"$SCRIPT_DIR/templates/WG07N1414_template.xlsm\" -a \"$SCRIPT_DIR/anchor_results/VPCC-NF.csv\" -p \"$csv_file\" -x \"$OUT/metrics/VPCC-NF-Anchor__vs__proposed.xlsm\"" >> "$RUN_SUMMARY"
fi

# --- Functions for parallel jobs (unchanged) ---
run_with_progress() {
    local jobfile="$1"
    local max_parallel="$2"
    local total
    total=$(wc -l < "$jobfile")
    local count=0
    # Default to nproc if not provided or invalid
    if [[ -z "$max_parallel" || "$max_parallel" -lt 1 ]]; then
        max_parallel=$(nproc)
    fi

    echo "Total jobs: $total"
    cat "$jobfile" | xargs -P "${max_parallel}" -I {} bash -c '{} ; echo DONE' |
    while read -r signal; do
        count=$((count+1))
        percent=$((count*100/total))
        echo -ne "Progress: $count/$total (${percent}%)\r"
    done
    echo
}

run_with_progress_sbatch_wait() {
    local jobfile="$1"
    local done_dir="$2"
    mkdir -p "$done_dir"

    local total
    total=$(wc -l < "$jobfile")
    local submitted=0
    local done_count=0
    echo "Submitting $total jobs to SLURM..."
    while read -r job; do
        while (( $submitted - $done_count > 10000 )); do
            sleep 4
            submit_percent=$((submitted*100/total))
            done_count=`ls $done_dir | wc -l`
            done_percent=$((done_count*100/total))
            echo -ne "Submited: $submitted/$total (${submit_percent}%). Progress: $done_count/$total (${done_percent}%)\r"
        done
        sbatch $job >/dev/null
        submitted=$((submitted+1))
        submit_percent=$((submitted*100/total))
        done_count=`ls $done_dir | wc -l`
        done_percent=$((done_count*100/total))
        echo -ne "Submited: $submitted/$total (${submit_percent}%). Progress: $done_count/$total (${done_percent}%)\r"
    done < "$jobfile"

    echo "All jobs submitted. Waiting for completion..."
    sleep 2

    while (( done_count < total )); do
        done_count=`ls $done_dir | wc -l`
        percent=$((done_count*100/total))
        echo -ne "Progress: $done_count/$total (${percent}%)\r"
        sleep 1
    done
    echo
}

## Master run scripts
MASTER_RUN_SLURM="$OUT/jobs/run_all_slurm.sh"
MASTER_RUN="$OUT/jobs/run_all.sh"

# SLURM version
declare -f run_with_progress_sbatch_wait > "$MASTER_RUN_SLURM"
{
    echo "#!/bin/bash"
    echo "set -euo pipefail"
    echo "cd $(dirname "$(readlink -f "$0" 2>/dev/null || realpath "$0")")"
    echo
    echo "echo '🔧 Detected nproc = $(nproc)'"
    echo
    echo "echo '▶ Step 1: VPCC encode/decode in parallel'"
    echo "run_with_progress_sbatch_wait \"$RUN_CODEC\" \"$OUT/jobs/done/codec/\" "
    echo
    echo "echo '▶ Step 2: Metric calculation in parallel'"
    echo "run_with_progress_sbatch_wait \"$RUN_METRIC\" \"$OUT/jobs/done/metric/\""
    echo
    echo "echo '▶ Step 3: Summary generation'"
    echo "bash \"$RUN_SUMMARY\""
    echo "echo '✅ All tasks completed'"
} >> "$MASTER_RUN_SLURM"
chmod +x "$MASTER_RUN_SLURM"

# Local version
declare -f run_with_progress > "$MASTER_RUN"
{
    echo "#!/bin/bash"
    echo "set -euo pipefail"
    echo "cd $(dirname "$(readlink -f "$0" 2>/dev/null || realpath "$0")")"
    echo
    echo "echo '🔧 Detected nproc = $(nproc)'"
    echo
    echo "echo '▶ Step 1: VPCC encode/decode in parallel'"
    echo "run_with_progress \"$RUN_CODEC\" \$(nproc)"
    echo
    echo "echo '▶ Step 2: Metric calculation in parallel'"
    echo "run_with_progress \"$RUN_METRIC\"" 1
    echo
    echo "echo '▶ Step 3: Summary generation'"
    echo "bash \"$RUN_SUMMARY\""
    echo "echo '✅ All tasks completed'"
} >> "$MASTER_RUN"
chmod +x "$MASTER_RUN"

echo
echo "[Generated scripts]"
echo "$RUN_CODEC"
echo "$RUN_METRIC"
echo "$RUN_SUMMARY"
echo "$MASTER_RUN"
echo "$MASTER_RUN_SLURM"
echo
echo "💡 Run (Local): bash \"$MASTER_RUN\""
echo "💡 Run (Slurm): bash \"$MASTER_RUN_SLURM\""

if [ $AUTO_RUN == 1 ]; then
    bash "$MASTER_RUN"
elif [ $AUTO_RUN == 2 ]; then
    bash "$MASTER_RUN_SLURM"
fi
