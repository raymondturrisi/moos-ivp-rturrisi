#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch_shoreside.sh    
#  Mission: rescue_baseline
#   Author: Michael Benjamin   
#   LastEd: April 2022       
#-------------------------------------------------------------- 
#  Part 1: Set global variables
#-------------------------------------------------------------- 
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
VERBOSE=""
CONFIRM="no"
AUTO_LAUNCHED="no"
CMD_ARGS=""

IP_ADDR="localhost"
MOOS_PORT="9000"
PSHARE_PORT="9200"
SWIM_FILE="mit_00.txt"

#--------------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#--------------------------------------------------------------
for ARGI; do
    CMD_ARGS+="${ARGI} "
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS] [time_warp]                       " 
	echo "                                                 "
	echo "Options:                                         "
        echo "  --help, -h                                     "
        echo "    Display this help message                    "
        echo "  --verbose, -v                                  "
        echo "    Increase verbosity                           "
	echo "  --noconfirm, -nc                               " 
	echo "    No confirmation before launching             "
	echo "  --just_make, -j                                " 
	echo "    Just make target files. Do not launch.       "
        echo "  --auto, -a                                     "
        echo "     Auto-launched by a script.                  "
        echo "     Will not launch uMAC as the final step.     "
	echo "                                                 "
	echo "  --ip=<localhost>                               " 
	echo "    Force pHostInfo to use this IP Address       "
	echo "  --mport=<9000>                                 "
	echo "    Port number of this vehicle's MOOSDB port    "
	echo "  --pshare=<9200>                                " 
	echo "    Port number of this vehicle's pShare port    "
	echo "                                                 "
	echo "  --sfile=<filename>                             " 
	echo "    Name of the swim file                        "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE="yes"
	CONFIRM="yes"
    elif [ "${ARGI}" = "--noconfirm" -o "${ARGI}" = "-nc" ]; then
	CONFIRM="no"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes"

    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:7}" = "--mport" ]; then
	MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"
    elif [ "${ARGI:0:12}" = "--swim_file=" ]; then
        SWIM_FILE="${ARGI#--swim_file=*}"
    elif [ "${ARGI}" = "-1" ]; then
        SWIM_FILE="mit_01.txt"
    elif [ "${ARGI}" = "-2" ]; then
        SWIM_FILE="mit_02.txt"

    else 
	echo "$ME: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

#---------------------------------------------------------------
#  Part 3: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" -o "${CONFIRM}" = "yes" ]; then 
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]"
    echo "TIME_WARP =     [${TIME_WARP}]"
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "MOOS_PORT =     [${MOOS_PORT}]"
    echo "PSHARE_PORT =   [${PSHARE_PORT}]"
    echo "IP_ADDR =       [${IP_ADDR}]"
    echo -n "Hit any key to continue with launching"
    read ANSWER
fi

#-------------------------------------------------------
#  Part 4: Create the .moos and .bhv files. 
#-------------------------------------------------------
nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP  \
       IP_ADDR=$IP_ADDR       PSHARE_PORT=$PSHARE_PORT             \
       MOOS_PORT=$MOOS_PORT   SWIM_FILE=$SWIM_FILE

if [ ${JUST_MAKE} = "yes" ]; then
    echo "$ME: Files assembled; nothing launched; exiting per request."
    exit 0
fi

#-------------------------------------------------------
#  Part 5: Launch the processes
#-------------------------------------------------------
echo "Launching Shoreside MOOS Community. WARP="$TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &
echo "Done Launching Shoreside Community"


#---------------------------------------------------------------
#  Part 6: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#---------------------------------------------------------------
# Part 7: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_shoreside.moos
kill -- -$$
