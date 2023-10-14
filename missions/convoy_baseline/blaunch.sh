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
TIME_WARP=20
PARAM_GENERATOR=""
PARAM_RANGE=""
PARAM_START=1
PARAM_END=1

TRIALS=1
POST_PROCESS_SCRIPT_PATH=""
LAUNCH_ARGS=""

idx=0
while [[ idx -lt $# ]]; do
    idx=$((idx+1))
    ARGI=${!idx}
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
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then
        LAUNCH_ARGS+=" $ARGI"
    else 
	echo "$ME: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

idx=0
p_pid=-1
t_start=$(date +%s)
logname="${t_start}.log"
for i in $(seq $CONFIG_START $CONFIG_END); do
    python3 $CONFIG_GENERATOR $i

    for j in $(seq $PARAM_START $PARAM_END); do
        python3 $PARAM_GENERATOR $j

        for k in $(seq 1 $TRIALS); do
            t_now=$(date +%s)
            duration=$((t_now-t_start))
            mission_name=$(printf "C%03d_P%05d_K%1d" $i $j $k)
            echo "${idx} | ${duration}: Running configuration ${i}, Parameter set ${j}, for trial ${k}"
            ./launch.sh $LAUNCH_ARGS $TIME_WARP --mname=$mission_name  >& /dev/null &
            pid_l=$!
	    
            sleep 8

            MOOSTime=$(date +%s)
            MOOSTime=$(( $MOOSTime * 10 + 5))

            uPokeDB targ_shoreside.moos \
                              DEPLOY_ALL=true \
                MOOS_MANUAL_OVERRIDE_ALL=false \
                              LOITER_ALL=false \
                              RETURN_ALL=false \
                             STATION_ALL=false \
                       MISSION_TASK_ALL=type=waypoint,id=001,waypt_x=20,waypt_y=-20,task_time=@MOOSTime \
                VIEW_RANGE_PULSE=x=0,y=0,radius=50,duration=10,fill=0.9,label=nil, \
                      edge_color=white,fill_color=white,edge_size=1 \
                &> /dev/null &

            DONE="false"

            while [ "${DONE}" = "false" ] ; do 
                if uQueryDB targ_shoreside.moos           \
                    --condition="QUIT_MISSION == true" ; then 
                echo "   Mission Complete" 
                DONE="true"
                elif uQueryDB targ_shoreside.moos         \
                    --condition="DB_UPTIME >= 600"  ; then 
                echo "   Mission TimeOut" 
                DONE="true"
                else
                echo "   Mission continuing..."
                sleep 5
                fi
            done

	    nuke_moos &
	    sleep 1 
	    nuke_moos &
	    #pkill -P $pid_l 
            #TODO: Check to make sure post process completes okay but still detatch/proceed
            ./post_process.sh $mission_name &
            #p_pid=$!
            rm targ_*
            t_now=$(date +%s)
            echo "$idx, $t_now" >> $logname
            sleep 2
            idx=$((idx+1))
        done

    done

done

#Need to wait for the last post processor to finish
wait $!
