#!/bin/bash -e
#--------------------------------------------------------------
#   Script: blaunch.sh  (Batch Launch)
#   Author: Raymond Turrisi
#     Date: Sept. 19
#--------------------------------------------------------------

# Given a directory of configuration conditions, and a directory of parameter conditions, each set of parameters for each set of configurations K times. 
# i.e. k*(configuration_conditions * parameter conditions)

# Initialize initial values for script

# Parse arguments

## Arguments being: 
# An index or an index range for a set of configuration conditions, for use with a generator function
# An index or an index range for a set of parameter conditions, for use with a generator function
# optionally: 
# - time warp
# - the number of trials for each pairing
# - whether or not a post processing script/job will be executed after each run
# i.e. ./blaunch.sh 10 --cc="config_set1.py" --cargs="[1..100]" --cp="param_set1.py" --pargs="[1..100]" --k=3 --pp="post_sequence.sh"
#   > batch launch at timewarp 10, with a generator function for configurations named config_set1.py, configuration function arguments 
#       (in this case, a range from indices 1 to 100), a parameter generator function named param_set1.py, with parameter configuration arguments, at 3 trials each, passing a post processing shell script

CONFIG_GENERATOR=""
CONFIG_RANGE=""
CONFIG_START=1
CONFIG_END=1

PARAM_GENERATOR=""
PARAM_RANGE=""
PARAM_START=1
PARAM_END=1

TRIALS=1
POST_PROCESS_SCRIPT_PATH=""

idx=0
while [[ idx -lt $# ]]; do
    idx=$((idx+1))
    ARGI=${!idx}
    echo $ARGI
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "Not yet!"
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--cg" ]; then
        idx=$((idx+1))
        CONFIG_GENERATOR=${!idx}
    elif [ "${ARGI}" = "--pg" ]; then
        idx=$((idx+1))
        PARAM_GENERATOR=${!idx}
    elif [ "${ARGI}" = "--k" ]; then
        idx=$((idx+1))
        TRIALS=${!idx}
    elif [ "${ARGI:0:9}" = "--crange=" ]; then
        TERM="${ARGI#--crange=*}"
        TERM="${TERM//[\[\]]}"
        CONFIG_START="${TERM%%..*}"
        CONFIG_END="${TERM##*..}"
    elif [ "${ARGI:0:9}" = "--prange=" ]; then
        TERM="${ARGI#--prange=*}"
        TERM="${TERM//[\[\]]}"
        PARAM_START="${TERM%%..*}"
        PARAM_END="${TERM##*..}"
    else 
	echo "$ME: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

idx=0
for i in $(seq $CONFIG_START $CONFIG_END); do
    for j in $(seq $PARAM_START $PARAM_END); do
        for k in $(seq 1 $TRIALS); do
            #DO STUFF HERE!
        done
    done
done

#for configuration ...
#    for parameter set ...
#        for i in k trials ...
#            ./launch.sh *args*
#            ...
#            while termination conditions are not met
#                watch for termination conditions
#                    break if met
#            given the mission data file, and if given a post process script, pass the mission data file to the post processing script