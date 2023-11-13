#!/bin/bash

#Missing From Batch
#Useage: mfb.sh --crange="[1..2]" --prange="[1..2848]" --k 2
# Initialize index for argument parsing
idx=0
DIRECTORY=""
while [[ idx -lt $# ]]; do
    idx=$((idx+1))
    ARGI=${!idx}
    if [ "${ARGI:0:9}" = "--crange=" ]; then
        TERM="${ARGI#--crange=*}"
        TERM="${TERM//[\[\]]}"
        CONFIG_START="${TERM%%..*}"
        CONFIG_END="${TERM##*..}"
    elif [ "${ARGI:0:9}" = "--prange=" ]; then
        TERM="${ARGI#--prange=*}"
        TERM="${TERM//[\[\]]}"
        PARAM_START="${TERM%%..*}"
        PARAM_END="${TERM##*..}"
    elif [ "${ARGI:0:6}" = "--dir=" ]; then
        DIRECTORY="${ARGI#--dir=*}"
    elif [ "${ARGI}" = "--k" ]; then
        #echo $idx
        idx=$((idx+1))
        #echo $idx
        TRIALS=${!idx}
    else 
        echo "Bad Arg: $ARGI. Exit Code 1."
        exit 1
    fi
done

# Ensure the directory for logs exists
# For each configuration index which was given
total=0
missing=0
for i in $(seq $CONFIG_START $CONFIG_END); do

    # For each parameter index which was given
    for j in $(seq $PARAM_START $PARAM_END); do

        # For each trial which was asked
        for k in $(seq 1 $TRIALS); do
            # Format numbers to match the filename pattern
            CG_NUM=$(printf "%03d" $i)
            P_NUM=$(printf "%05d" $j)
            K_NUM=$k
            # Construct filename
            FILENAME="${DIRECTORY}C${CG_NUM}_P${P_NUM}_K${K_NUM}_meta"
            # Check if file exists, if not, log the filename
            if [[ ! -d $FILENAME ]]; then
                echo "$FILENAME" 
                missing=$((missing+1))
            fi
            total=$((total+1))
        done

    done

done

echo "#n missing files        = $missing"
echo "#n total files in span  = $total"