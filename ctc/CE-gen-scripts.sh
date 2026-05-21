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
declare -A UNCOMPRESSED_DATA_PATHs
declare -A COLMAP_DIRs
declare -A RESOLUTIONS
declare -A START_FRAME_NR
declare -A AVAILABLE_FRAMES

# ==================================================================================================
# Experiment settings
# ==================================================================================================

JOBNAME="V-PCC-GS"
AUTO_RUN=0                  # 0:off, 1:run_local, 2:run_slurm 
LENGTHS=(1F)                # Valid values for LENGTHS are (1F), (NF), or (1F NF)
CODINGS=(C1 CW)             # Valid values for CODINGS are (C1), (CW), or (C1 CW)
KEEP_INTERMEDIATE_FILES=0   # 0: reduce diskspace, or 1: keep intermediate files
GENERATE_PNGs=0             # 0 or 1
: "${EXPERIMENT_MODE:=2}"   # 0: mandatory, 1: all, 2: man_with_fruit_tracked only, 3: subset
: "${METRICS_THREADS:=1}"   # Number of threads for mpeg-gsc-metrics (optimize for your system resources)

# ==================================================================================================
# Coding settings
# ==================================================================================================

MAPMODE=1                   # 0: INTERLEAVED, 1: PLANAR(CTC)
PACKING_MODE=1              # 0: pack-none, 1: pack-attr, 2: pack-all
IMPLICIT_PACKING=1          # 0: OFF(explicit), 1:ON (implicit)
SH_MAPPING=1                # set to 0 to use interleaved maping in the video bitstream (1 corresponds to CTC, i.e. planar mapping)
COLMAP_SEI=0                # set it to 1 if you want to send the camera pareters as an SEI message
GSD_TRANSFORM_2D=1          # 1: enable 2D transform for sphereical harmnoics, 0: disable
GSD_ADAPTIVE_SH=1           # when GSD_TRANSFORM_2D=1, 1: enable m75790, 0: enable m75380
GSD_NON_UNIFORM_QUANT=1     # 1: enable m75025, 0: disable m75025
ADAPTIVE_QUANTIZATION=1     # 1: enable m75039, 0: disable
ANNOTATE_DIR=""             # m75039: path to directory of importance score files for each sequence. If string is empty, files are recomputed

# ==================================================================================================
# Base directories
#
# Use environment variables to override default values
# ==================================================================================================

: "${DATASET_DIR:=/path/to/CTC}"
: "${METRIC_DIR:=/path/to/mpeg-gsc-metrics/build/Release/bin}"
: "${GSCTOOL_DIR:=/path/to/mpeg-gsc-tools}"
: "${CAMERAPOS_DIR:=$GSCTOOL_DIR/gsTools/build}"
: "${TMC_DIR:=/path/to/mpeg-pcc-tmc2}"
: "${BASE_OUTPUT:=/path/to/CE}"
: "${TIME:=/bin/time }"
: "${PYTHON:=python3}"

SCRIPT_PATH="$(readlink -f "$0" 2>/dev/null || realpath "$0")"
: "${SCRIPT_DIR:=$(dirname "$SCRIPT_PATH")}"

# ==================================================================================================
# Datasets
# ==================================================================================================

SEQUENCES=(
 "bartender_tracked"
 "bartender_semitracked"
 "cinema_tracked"
 "cinema_semitracked"
 "breakfast_tracked"
 "breakfast_semitracked"
 "breakfast_untracked"
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

# ==================================================================================================
# Enable sequences for the experiment
# ==================================================================================================

# By default mandatory sequences are enabled (1), and optional sequences are disabled (0). Currently
# there are no optional sequences in the CTC, but that may change, so do not simplify this part:
ENABLE_SEQs[bartender_tracked]=1
ENABLE_SEQs[bartender_semitracked]=1
ENABLE_SEQs[cinema_tracked]=1
ENABLE_SEQs[cinema_semitracked]=1
ENABLE_SEQs[breakfast_tracked]=1
ENABLE_SEQs[breakfast_semitracked]=1
ENABLE_SEQs[breakfast_untracked]=1
ENABLE_SEQs[trio]=1
ENABLE_SEQs[makeup]=1
ENABLE_SEQs[musical]=1
ENABLE_SEQs[manwithfruit_tracked]=1
ENABLE_SEQs[lego_ferrari]=1
ENABLE_SEQs[lego_bugatti]=1
ENABLE_SEQs[cricket_player]=1
ENABLE_SEQs[plant]=1
ENABLE_SEQs[solo_tango_female]=1
ENABLE_SEQs[solo_tango_male]=1
ENABLE_SEQs[tango_duo]=1
ENABLE_SEQs[tennis_player]=1
ENABLE_SEQs[library]=1
ENABLE_SEQs[flowerdance]=1
ENABLE_SEQs[gymnast]=1

if [ $EXPERIMENT_MODE -eq 1 ]; then
    for SEQ in "${!ENABLE_SEQs[@]}"; do
        ENABLE_SEQs[$SEQ]=1
    done
    JOBNAME="$JOBNAME-all"
elif [ $EXPERIMENT_MODE -eq 2 ]; then
    for SEQ in "${!ENABLE_SEQs[@]}"; do
        ENABLE_SEQs[$SEQ]=0
    done
    ENABLE_SEQs[manwithfruit_tracked]=1
    JOBNAME="$JOBNAME-manwithfruit"
elif [ $EXPERIMENT_MODE -eq 3 ]; then
    ENABLE_SEQs[musical]=0
    ENABLE_SEQs[library]=0
    ENABLE_SEQs[flowerdance]=0
    ENABLE_SEQs[gymnast]=0
    JOBNAME="$JOBNAME-subset"
fi

# ==================================================================================================
# Source data locations (uncompressed PLYs and COLMAP data locatiosn per JEE 6.1)
# ==================================================================================================

for SEQ in "${SEQUENCES[@]}"; do
    UNCOMPRESSED_DATA_PATHs[$SEQ]="$DATASET_DIR/$SEQ/plys/frame_%04i.ply"
    COLMAP_DIRs[$SEQ]="$DATASET_DIR/$SEQ/colmap_data"
done

# ==================================================================================================
# Start frame numbers and available frames (per CTC)
# ==================================================================================================

for SEQ in "${SEQUENCES[@]}"; do
    START_FRAME_NR[$SEQ]=0
    AVAILABLE_FRAMES[$SEQ]=1
done

START_FRAME_NR[manwithfruit_tracked]=81
START_FRAME_NR[gymnast]=200

AVAILABLE_FRAMES[bartender_tracked]=32
AVAILABLE_FRAMES[bartender_semitracked]=32
AVAILABLE_FRAMES[cinema_tracked]=32
AVAILABLE_FRAMES[cinema_semitracked]=32
AVAILABLE_FRAMES[breakfast_tracked]=32
AVAILABLE_FRAMES[breakfast_semitracked]=32
AVAILABLE_FRAMES[breakfast_untracked]=32
AVAILABLE_FRAMES[manwithfruit_tracked]=32

# ==================================================================================================
# Reference view resolutions (per CTC)
# ==================================================================================================

RESOLUTIONS[bartender_tracked]="1920x1080"
RESOLUTIONS[bartender_semitracked]="1920x1080"
RESOLUTIONS[cinema_tracked]="1920x1080"
RESOLUTIONS[cinema_semitracked]="1920x1080"
RESOLUTIONS[breakfast_tracked]="1920x1080"
RESOLUTIONS[breakfast_semitracked]="1920x1080"
RESOLUTIONS[breakfast_untracked]="1920x1080"
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

# ==================================================================================================
# Calculate derived values
# ==================================================================================================

METRIC="$METRIC_DIR/mpeg-gsc-metrics"
CAMERAPOS_BIN="$CAMERAPOS_DIR/cameraPosition"
CFG_PATH="$TMC_DIR/cfg/gs4"
IMP_BIN="$TMC_DIR/build/bin/importance-scores"
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

# ==================================================================================================
# Validate input files
# ==================================================================================================

for SEQ in "${SEQUENCES[@]}"; do
    if [ "${ENABLE_SEQs[$SEQ]}" -ne 1 ]; then
        echo "⏭️ Skipping disabled $SEQ"
        continue
    fi
    REQUIRED_FRAMES=0

    for LENGTH in "${LENGTHS[@]}"; do
        case $LENGTH in
            "1F") if (( REQUIRED_FRAMES < 1 )); then REQUIRED_FRAMES=1; fi ;;
            "NF") if (( REQUIRED_FRAMES < 32 && 32 <= AVAILABLE_FRAMES[$SEQ] )); then REQUIRED_FRAMES=32; fi ;;
            *) echo "Unknown length variant: $LENGTH"; exit 1 ;;
        esac
    done
    for ((i = START_FRAME_NR[$SEQ]; i < START_FRAME_NR[$SEQ] + REQUIRED_FRAMES; i++)); do
        FILE=$(printf "${UNCOMPRESSED_DATA_PATHs[$SEQ]}" $i)
        if [[ ! -f "$FILE" ]]; then
            echo "❌ Failed to validate $SEQ due to missing file: $FILE"
            exit 1
        fi
    done
    echo "✅ Validated ${REQUIRED_FRAMES} frames of $SEQ"
done

# ==================================================================================================
# Create output directories and copy binaries
# ==================================================================================================

CURTIME="$(date "+%y-%m-%d-%H%M%S")"
OUT="$BASE_OUTPUT/$JOBNAME-$CURTIME"
BATCH_ANNOTATE="$OUT/jobs/batch_annotate"
BATCH_CODEC="$OUT/jobs/batch_codec"
BATCH_METRIC="$OUT/jobs/batch_metric"
LOG_DIR="$OUT/jobs/logs"

MKDIRCK() { [[ -d "$1" ]] || mkdir -p "$1"; }

MKDIRCK "$OUT/metrics"
MKDIRCK "$BATCH_CODEC"
MKDIRCK "$BATCH_METRIC"
MKDIRCK "$OUT/bin"
MKDIRCK "$LOG_DIR"
MKDIRCK "$OUT/jobs/done/codec"
MKDIRCK "$OUT/jobs/done/metric"

if [ -z "$ANNOTATE_DIR" ] && [ $ADAPTIVE_QUANTIZATION == 1 ]; then
    MKDIRCK "$BATCH_ANNOTATE"
    MKDIRCK "$OUT/jobs/done/annotate"
fi

cp "$TMC_ENC_BIN" "$OUT/bin/$(basename $TMC_ENC_BIN)"
if [ $TMC_ENC_BIN != $TMC_DEC_BIN ]; then
    cp "$TMC_DEC_BIN" "$OUT/bin/$(basename $TMC_DEC_BIN)"
fi

TMC_ENC_BIN="$OUT/bin/$(basename $TMC_ENC_BIN)"
TMC_DEC_BIN="$OUT/bin/$(basename $TMC_DEC_BIN)"

cp "$CAMERAPOS_BIN" "$OUT/bin/$(basename $CAMERAPOS_BIN)"
CAMERAPOS_BIN="$OUT/bin/$(basename $CAMERAPOS_BIN)"

if [ -f "$IMP_BIN" ]; then
    cp "$IMP_BIN" "$OUT/bin/$(basename $IMP_BIN)"
    IMP_BIN="$OUT/bin/$(basename $IMP_BIN)"
fi

cd "$OUT/jobs"
RUN_ANNOTATE="$OUT/jobs/step0_run_annotate.sh"
RUN_CODEC="$OUT/jobs/step1_run_codec.sh"
RUN_METRIC="$OUT/jobs/step2_run_metric.sh"
RUN_SUMMARY="$OUT/jobs/step3_run_summary.sh"

# ==================================================================================================
# Generate the scripts
# ==================================================================================================

: > "$RUN_CODEC"
: > "$RUN_METRIC"
: > "$RUN_SUMMARY"

csv_file="$OUT/metrics/$JOBNAME.csv"

echo "$PYTHON $PARSER_SCRIPT --print-header-row > \"$csv_file\"" >> "$RUN_SUMMARY"

for SEQ in "${SEQUENCES[@]}"; do
    if [ "${ENABLE_SEQs[$SEQ]}" -ne 1 ]; then
        echo "⏭️ Skipping disabled $SEQ"
        continue
    fi

    REN_W=$(echo ${RESOLUTIONS[$SEQ]} | cut -d'x' -f1)
    REN_H=$(echo ${RESOLUTIONS[$SEQ]} | cut -d'x' -f2)

    for LENGTH in "${LENGTHS[@]}"; do
        case $LENGTH in
            "1F") FRAME_COUNT=1 ; METRIC_CPU=1 ;;
            "NF") FRAME_COUNT=32 ; METRIC_CPU=1 ;;
            *) echo "Unknown length variant: $LENGTH"; exit 1 ;;
        esac
        if [ "${AVAILABLE_FRAMES[$SEQ]}" -lt "$FRAME_COUNT" ]; then
            continue  # Not enough available frames
        fi
        
        echo
        echo "Processing sequence $LENGTH $SEQ"
        echo "Printing the md5sum of the first input file:"
        md5sum $(printf ${UNCOMPRESSED_DATA_PATHs[$SEQ]} ${START_FRAME_NR[$SEQ]})
        
        if [ -z "$ANNOTATE_DIR" ] && [ $ADAPTIVE_QUANTIZATION == 1 ]; then
            SEQ_GROUP_DIR="$OUT/importance_scores/$LENGTH/$SEQ"
            MKDIRCK "$SEQ_GROUP_DIR"
            
            for ((i = START_FRAME_NR[$SEQ]; i < START_FRAME_NR[$SEQ] + FRAME_COUNT; i++)); do
                INPUT_FILE=$(printf "${UNCOMPRESSED_DATA_PATHs[$SEQ]}" $i)
                FRAME_NR=$(printf "%04i" $i)
                FRAME="frame_$FRAME_NR"

                # Compute importance-score based quantization groups
                JOBFILE_ANO="$BATCH_ANNOTATE/$JOBNAME-$CURTIME-$LENGTH-$SEQ-$FRAME.sh"
                {
                    echo "#!/bin/bash"
                    echo "#SBATCH --output=$OUT/jobs/logs/%j.out"
                    echo "#SBATCH --ntasks-per-node=1"
                    echo "#SBATCH --nodes=1"
                    echo "#SBATCH --mem-per-cpu=8G"
                    echo "#SBATCH --cpus-per-task=$METRICS_THREADS"
                    echo ""
                    echo "# Printing the md5sum of the input PLY for debugging"
                    echo "md5sum $INPUT_FILE"
                    echo ""
                    echo "# Add camera header to source"
                    echo "$CAMERAPOS_BIN \\"
                    echo "    --input=\"$INPUT_FILE\" \\"
                    echo "    --output=\"$SEQ_GROUP_DIR/$FRAME.camerpos.ply\" \\"
                    echo "    --image=\"${COLMAP_DIRs[$SEQ]}/$FRAME/sparse/images.bin\" \\"
                    echo "    --camera=\"${COLMAP_DIRs[$SEQ]}/$FRAME/sparse/cameras.bin\""
                    echo
                    echo "# Calculate importance scores"
                    echo "$TIME$IMP_BIN --threads=$METRICS_THREADS \\"
                    echo "    -a \"$SEQ_GROUP_DIR/$FRAME.camerpos.ply\" \\"
                    echo "    \"--impScore=$SEQ_GROUP_DIR/$FRAME.impScores.txt\" \\"
                    echo "    \"--width=$REN_W\" \"--height=$REN_H\" "
                    echo "touch $OUT/jobs/done/annotate/\$SLURM_JOB_ID"
                } > "$JOBFILE_ANO"
                chmod +x "$JOBFILE_ANO"
                echo "$JOBFILE_ANO > $LOG_DIR/$JOBNAME-$CURTIME-$LENGTH-$SEQ-$FRAME.annotate.log " >> "$RUN_ANNOTATE"
            done
        else
            SEQ_GROUP_DIR="$ANNOTATE_DIR/$LENGTH/$SEQ"
        fi

        echo "Configuration path: ${CFG_PATH}"
        CFG_RATE_DIRs=($CFG_PATH/rate/*)

        for CFG_RATE_DIR in "${CFG_RATE_DIRs[@]}"; do
            RATE="$(basename "$CFG_RATE_DIR")"

            if [ "$RATE" = "lossless" ] && [[ ! " ${CODINGS[@]} " =~ "CW" ]]; then
                echo Condition CW is disabled
                continue
            fi
            if [ "$RATE" != "lossless" ] && [[ ! " ${CODINGS[@]} " =~ "C1" ]]; then
                echo Condition C1 is disabled
                continue
            fi

            RESULT_DIR="$OUT/results/$LENGTH/$SEQ/$RATE"
            MKDIRCK "$RESULT_DIR/plys"
            MKDIRCK "$RESULT_DIR/pos_plys"
            MKDIRCK "$RESULT_DIR/metrics"

            echo "Rate point: $SEQ/$RATE"

            JOBFILE="$BATCH_CODEC/$JOBNAME-$CURTIME-$LENGTH-$SEQ-$RATE.sh"
            {
                echo "#!/bin/bash"
                echo "#SBATCH --output=$OUT/jobs/logs/codec/%j-$LENGTH-$SEQ-$RATE.out"
                echo "#SBATCH --ntasks-per-node=1"
                echo "#SBATCH --nodes=1"
                echo "#SBATCH --mem-per-cpu=8G"
                echo ""
                echo "# Encode all $FRAME_COUNT frames in one invocation"
                echo "$TIME$TMC_ENC_BIN \\"
                echo "    --uncompressedDataPath=${UNCOMPRESSED_DATA_PATHs[$SEQ]} \\"
                echo "    --configurationFolder=$CFG_PATH/ \\"
                echo "    --config=$CFG_PATH/gs-main10-common.cfg \\"
                echo "    --config=$PACK_CONFIG \\"
                if (( $ADAPTIVE_QUANTIZATION == 1 )); then
                    FRAME="$(printf "frame_%04i" ${START_FRAME_NR[$SEQ]})"
                    echo "    --config=$CFG_PATH/gs-adaptive-quantization.cfg \\"
                    echo "    --importanceScoresPath=\"$SEQ_GROUP_DIR/$FRAME.impScores.txt\" \\"
                fi
                echo "    --config=$CFG_RATE_DIR/encoder.cfg \\"
                echo "    --startFrameNumber=${START_FRAME_NR[$SEQ]} \\"
                echo "    --frameCount=$FRAME_COUNT \\"
                if [ $KEEP_INTERMEDIATE_FILES -eq 1 ]; then
                    MKDIRCK "$RESULT_DIR/debug"
                    echo "    --keepIntermediateFiles=1 --debugPath=$RESULT_DIR/debug \\"
                fi
                if (( $SH_MAPPING == 0 )); then
                    echo "    --shMappingMode=0 \\"
                fi
                if (( $COLMAP_SEI == 1 )); then
                    FIRST_FRAME="$(printf "frame_%04i" ${START_FRAME_NR[$SEQ]})"
                    echo "    --GSColmapFolderPath=\"${COLMAP_DIRs[$SEQ]}/$FIRST_FRAME/sparse/\" \\"
                fi
                if [ "$PACKING_MODE" -ne 0 ]; then
                    echo "    --implicitPacking=$IMPLICIT_PACKING \\"
                fi
                echo "    --gsdTransform2dPresentFlag=$GSD_TRANSFORM_2D \\"
                echo "    --dataAdaptiveShTransformEnabled=$GSD_ADAPTIVE_SH \\"
                echo "    --enableShNonUniformQuantization=$GSD_NON_UNIFORM_QUANT \\"
                echo "    --compressedStreamPath=$RESULT_DIR/${LENGTH}_${SEQ}_${RATE}.bin \\"
                echo "    2>&1 | tee $RESULT_DIR/${LENGTH}_${SEQ}_${RATE}.encoder.log"
                echo ""
                echo "# Decode all frames from single bitstream"
                echo "$TIME$TMC_DEC_BIN \\"
                echo "    --compressedStreamPath=$RESULT_DIR/${LENGTH}_${SEQ}_${RATE}.bin \\"
                echo "    --reconstructedDataPath=$RESULT_DIR/plys/${LENGTH}_${SEQ}_${RATE}_%04i.decoded.ply \\"
                echo "    --computeMetrics=0 --computeChecksum=0 \\"
                if [ $KEEP_INTERMEDIATE_FILES -eq 1 ]; then
                    MKDIRCK "$RESULT_DIR/debug"
                    echo "    --keepIntermediateFiles=1 --debugPath=$RESULT_DIR/debug \\"                    
                fi
                echo "    --startFrameNumber=${START_FRAME_NR[$SEQ]} \\"
                echo "    2>&1 | tee $RESULT_DIR/${LENGTH}_${SEQ}_${RATE}.decoder.log"
                echo ""
                echo "touch $OUT/jobs/done/codec/\$SLURM_JOB_ID"
            } > "$JOBFILE"
            chmod +x "$JOBFILE"
            echo "$JOBFILE > $LOG_DIR/$JOBNAME-$CURTIME-$LENGTH-$SEQ-$RATE.codec.log " >> "$RUN_CODEC"

            # Metrics are run per frame to allow for parallel execution
            for ((i = START_FRAME_NR[$SEQ]; i < START_FRAME_NR[$SEQ] + FRAME_COUNT; i++)); do
                INPUT_FILE=$(printf "${UNCOMPRESSED_DATA_PATHs[$SEQ]}" $i)
                FRAME_NR=$(printf "%04i" $i)
                FRAME="frame_$FRAME_NR"

                # Metrics per decoded frame
                JOBFILE_MET="$BATCH_METRIC/$JOBNAME-$CURTIME-$LENGTH-$SEQ-$RATE-$FRAME_NR.sh"
                {
                    echo "#!/bin/bash"
                    echo "#SBATCH --output=$OUT/jobs/logs/metric/%j-$LENGTH-$SEQ-$RATE-$FRAME_NR.out"
                    echo "#SBATCH --ntasks-per-node=1"
                    echo "#SBATCH --nodes=1"
                    echo "#SBATCH --mem-per-cpu=8G"
                    echo "#SBATCH --cpus-per-task=$METRICS_THREADS"
                    
                    if [ $METRIC_CPU -eq 0 ]; then
                        echo "#SBATCH --gpus=1"
                    fi

                    echo "# Add camera header to source"
                    echo "$CAMERAPOS_BIN \\"
                    echo "    --input=\"$INPUT_FILE\" \\"
                    echo "    --output=\"$RESULT_DIR/pos_plys/$FRAME.camerpos.ply\" \\"
                    echo "    --image=\"${COLMAP_DIRs[$SEQ]}/$FRAME/sparse/images.bin\" \\"
                    echo "    --camera=\"${COLMAP_DIRs[$SEQ]}/$FRAME/sparse/cameras.bin\""
                    echo
                    echo "# Calculate all metrics"
                    echo "$TIME$METRIC -v 1 --startFrame=0 --frameCount=1 --cpu=$METRIC_CPU --useCameraPosition=1 --threads=$METRICS_THREADS \\"
                    echo "    --width=$REN_W --height=$REN_H \\"
                    echo "    --source=\"$RESULT_DIR/pos_plys/$FRAME.camerpos.ply\" \\"
                    echo "    --decode=\"$RESULT_DIR/plys/${LENGTH}_${SEQ}_${RATE}_${FRAME_NR}.decoded.ply\" \\"
                    echo "    --output=\"$RESULT_DIR/metrics/${LENGTH}_${SEQ}_${RATE}_${FRAME_NR}.decoded.ply.metric\""

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
                echo "$JOBFILE_MET > $LOG_DIR/$JOBNAME-$CURTIME-$LENGTH-$SEQ-$RATE-$FRAME.metric.log" >> "$RUN_METRIC"
            done

            echo "$PYTHON $PARSER_SCRIPT -d \"$RESULT_DIR\" -l \"$LENGTH\" -s \"$SEQ\" -r \"$RATE\" -f ${START_FRAME_NR[$SEQ]} -n $FRAME_COUNT >> \"$csv_file\"" >> "$RUN_SUMMARY"
        done
        echo
    done
done

echo "$PYTHON $SCRIPT_DIR/gsc_csv2excel.py -t \"$SCRIPT_DIR/templates/WG04N0826_template.xlsm\" -a \"$SCRIPT_DIR/anchor_results/VPCC-GS.csv\" -p \"$csv_file\" -x \"$OUT/metrics/VPCC-GS-Anchor__vs__proposed.xlsm\"" >> "$RUN_SUMMARY"

# ==================================================================================================
# Functions for parallel jobs
# ==================================================================================================

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
{
    echo "#!/bin/bash"
    echo "set -euo pipefail"
    echo
} > "$MASTER_RUN_SLURM"
declare -f run_with_progress_sbatch_wait >> "$MASTER_RUN_SLURM"
{
    echo
    echo "cd $(dirname "$(readlink -f "$0" 2>/dev/null || realpath "$0")")"
    echo
    echo "echo '🔧 Detected nproc = $(nproc)'"
    echo
    if [ -z $ANNOTATE_DIR ] && [ $ADAPTIVE_QUANTIZATION == 1 ]; then
        echo "echo '▶ Step 0: ANNOTATE_DIR not set -> Run importance score computation in parallel'"
        echo "run_with_progress_sbatch_wait \"$RUN_ANNOTATE\" \"$OUT/jobs/done/annotate/\""
        echo
    fi
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
{
    echo "#!/bin/bash"
    echo "set -euo pipefail"
    echo
} > "$MASTER_RUN"
declare -f run_with_progress >> "$MASTER_RUN"
{
    echo
    echo "cd $(dirname "$(readlink -f "$0" 2>/dev/null || realpath "$0")")"
    echo
    echo "echo '🔧 Detected nproc = $(nproc)'"
    echo
    if [ -z $ANNOTATE_DIR ] && [ $ADAPTIVE_QUANTIZATION == 1 ]; then
        echo "echo '▶ Step 0: ANNOTATE_DIR not set -> importance score calculation in parallel'"
        echo "run_with_progress \"$RUN_ANNOTATE\"" 1
        echo
    fi
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

# ==================================================================================================
# Summary
# ==================================================================================================

echo
echo "[Generated scripts]"
if [ -z $ANNOTATE_DIR ] && [ $ADAPTIVE_QUANTIZATION == 1 ]; then
    echo "$RUN_ANNOTATE"
fi
echo "$RUN_CODEC"
echo "$RUN_METRIC"
echo "$RUN_SUMMARY"
echo "$MASTER_RUN"
echo "$MASTER_RUN_SLURM"
echo
echo "[Experiment settings]"
echo "Job name: $JOBNAME"
echo "Auto run: $AUTO_RUN" 
echo "Lengths: ${LENGTHS[*]}"
echo "Codings: ${CODINGS[*]}"
echo "Keep intermediate files: $KEEP_INTERMEDIATE_FILES"
echo "Generate PNGs: $GENERATE_PNGs"
echo "Experiment mode: $EXPERIMENT_MODE"
echo
echo "💡 Run (Local): bash \"$MASTER_RUN\""
echo "💡 Run (Slurm): bash \"$MASTER_RUN_SLURM\""

# ==================================================================================================
# Auto run (if enabled)
# ==================================================================================================

if [ $AUTO_RUN == 1 ]; then
    bash "$MASTER_RUN"
elif [ $AUTO_RUN == 2 ]; then
    bash "$MASTER_RUN_SLURM"
fi
