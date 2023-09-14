#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch_shoreside.sh    
#  Mission: juan_de_fuca
#   Author: Michael Benjamin   
#   LastEd: January 2021
#-------------------------------------------------------------- 
#  Part 1: Set global var defaults
#-------------------------------------------------------------- 
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
VERBOSE="no"
CONFIRM="yes"
AUTO_LAUNCHED="no"
LAUNCH_GUI="yes"
CMD_ARGS=""

IP_ADDR="localhost"
PSHARE_PORT="9200"
VNAMES=""

#-------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS] [time_warp]                      " 
        echo "  --help, -h                                     "
        echo "    Display this help message                    "
        echo "  --just_make, -j                                "
        echo "    Just make targ files, but do not launch      "
        echo "  --verbose, -v                                  "
        echo "    Verbose output, confirm before launching.    "
        echo "  --noconfirm, -nc                               "
        echo "    No confirmation before launching             "
        echo "  --auto, -a                                     "
        echo "     Auto-launched by a script.                  "
        echo "     Will not launch uMAC as the final step.     "
        echo "  --nogui, -n                                    "
        echo "     Headless mode - no pMarineViewer etc        "
        echo "                                                 "
        echo "  --ip=<localhost>                               "
        echo "    Force pHostInfo to use this IP Address       "
        echo "  --pshare=<9200>                                "
        echo "    Port number of this vehicle's pShare port    "
        echo "  --vnames=<vnames>                              "
        echo "    Colon-separate list of all vehicle names     "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI}" = "--noconfirm" -o "${ARGI}" = "-nc" ]; then
        CONFIRM="no"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes"
        CONFIRM="no"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then
        LAUNCH_GUI="no"

    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"
    elif [ "${ARGI:0:9}" = "--vnames=" ]; then
        VNAMES="${ARGI#--vnames=*}"
    else
        echo "$ME Bad Arg:" $ARGI " Exit Code 1"
        exit 1
    fi
done

#---------------------------------------------------------------
#  Part 3: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" -o "${CONFIRM}" = "yes" ]; then 
    echo "======================================================"
    echo "              launch_shoreside.sh SUMMARY             "
    echo "======================================================"
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]"
    echo "TIME_WARP =     [${TIME_WARP}]"
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "IP_ADDR =       [${IP_ADDR}]"
    echo "PSHARE_PORT =   [${PSHARE_PORT}]"
    echo "VNAMES=         [${VNAMES}]"
    echo -n "Hit the RETURN key to continue with launching"
    read ANSWER
fi

#---------------------------------------------------------------
#  Part 4: Create the .moos file.
#---------------------------------------------------------------
nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
       PSHARE_PORT=$PSHARE_PORT     VNAMES=$VNAMES                \
       IP_ADDR=$IP_ADDR             LAUNCH_GUI=$LAUNCH_GUI       

if [ ${JUST_MAKE} = "yes" ]; then
    echo "Files assembled; No launches; exiting per request."
    exit 0
fi

#-------------------------------------------------------
#  Part 5: Launch the processes
#-------------------------------------------------------
echo "Launching shoreside MOOS Community. WARP is:" $TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &


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
