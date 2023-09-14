#!/bin/bash
#-------------------------------------------------------------- 
#   Script: launch_vehicle.sh    
#  Mission: convoy_ct
#   Author: Michael Benjamin   
#   LastEd: November 2021
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#-------------------------------------------------------------- 
#  Part 2: Set Global Variables
#-------------------------------------------------------------- 

ME=`basename "$0"`
TIME_WARP=1
VERBOSE=""
JUST_MAKE="no"
CONFIRM="yes"
AUTO_LAUNCHED="no"
CMD_ARGS=""

IP_ADDR="localhost"
MOOS_PORT="9001"
PSHARE_PORT="9201"

SHORE_IP="192.168.1.224"
SHORE_PSHARE="9200"
VNAME=""
COLOR="gray70"
INDEX="1"

REGION="pavlab"
START_POS="0,0,180"  
RETURN_POS="5,0"
LOITER_SPD="3.0"  
MAX_SPD="5.0"

#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS] [time_warp]                       "
	echo "                                                 "
	echo "Options:                                         "
	echo "  --help, -h                                     " 
	echo "    Print this help message and exit             "
	echo "  --just_make, -j                                " 
	echo "    Just make targ files, but do not launch      "
	echo "  --verbose, -v                                  " 
	echo "    Verbose output, confirm before launching     "
	echo "  --noconfirm, -nc                               " 
	echo "    No confirmation before launching             "
        echo "  --auto, -a                                     "
        echo "     Auto-launched by a script.                  "
        echo "     Will not launch uMAC as the final step.     "
	echo "                                                 "
	echo "  --ip=<localhost>                               " 
	echo "    Force pHostInfo to use this IP Address       "
	echo "  --mport=<9001>                                 "
	echo "    Port number of this vehicle's MOOSDB port    "
	echo "  --pshare=<9201>                                " 
	echo "    Port number of this vehicle's pShare port    "
	echo "                                                 "
	echo "  --shore=<localhost>                            " 
	echo "    IP address location of shoreside             "
	echo "  --shore_pshare=<9200>                          " 
	echo "    Port on which shoreside pShare is listening  "
	echo "  --vname=<abe>                                  " 
	echo "    Name of the vehicle being launched           " 
	echo "  --index=<1>                                    " 
	echo "    Index for setting MOOSDB and pShare ports    "
	echo "                                                 "
	echo "  --start=<X,Y,H>     (default is 0,0,180)       " 
	echo "    Start position chosen by script launching    "
	echo "    this script (to ensure separation)           "
	echo "  --speed=meters/sec                             " 
	echo "    The speed use for transiting/loitering       "
	echo "                                                 "
	echo "  --maxspd=meters/sec                            " 
	echo "    Max speed of vehicle (for sim and in-field)  "
	echo "                                                 "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
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

    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHORE_IP="${ARGI#--shore=*}"
    elif [ "${ARGI:0:15}" = "--shore_pshare=" ]; then
        SHORE_PSHARE="${ARGI#--shore_pshare=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--color=" ]; then
        COLOR="${ARGI#--color=*}"
    elif [ "${ARGI:0:8}" = "--index=" ]; then
        INDEX="${ARGI#--index=*}"

    elif [ "${ARGI:0:8}" = "--start=" ]; then
        START_POS="${ARGI#--start=*}"
    elif [ "${ARGI:0:8}" = "--speed=" ]; then
        LOITER_SPD="${ARGI#--speed=*}"
    elif [ "${ARGI:0:9}" = "--maxspd=" ]; then
        MAX_SPD="${ARGI#--maxspd=*}"

    else 
	echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
	exit 1
    fi
done

MOOS_PORT=`expr $INDEX + 9050`
PSHARE_PORT=`expr $INDEX + 9250`

#---------------------------------------------------------------
#  Part 4: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" -o "${CONFIRM}" = "yes" ]; then 
    echo "======================================================"
    echo "       launch_vehicle_cn.sh SUMMARY            cn=$VNAME "
    echo "======================================================"
    echo "$ME"
    echo "CMD_ARGS =       [${CMD_ARGS}]      "
    echo "TIME_WARP =      [${TIME_WARP}]     "
    echo "AUTO_LAUNCHED =  [${AUTO_LAUNCHED}] "
    echo "----------------------------------  "
    echo "MOOS_PORT =      [${MOOS_PORT}]     "
    echo "PSHARE_PORT =    [${PSHARE_PORT}]   "
    echo "IP_ADDR =        [${IP_ADDR}]       "
    echo "----------------------------------  "
    echo "SHORE_IP =       [${SHORE_IP}]      "
    echo "SHORE_PSHARE =   [${SHORE_PSHARE}]  "
    echo "VNAME =          [${VNAME}]         "
    echo "COLOR =          [${COLOR}]         "
    echo "INDEX =          [${INDEX}]         "
    echo "----------------------------------  "
    echo "START_POS =      [${START_POS}]     "
    echo "LOITER_SPD =     [${LOITER_SPD}]    "
    echo "MAX_SPD =        [${MAX_SPD}]       "
    echo -n "Hit the RETURN key to continue with launching"
    read ANSWER
fi

#-------------------------------------------------------
#  Part 5: Create the .moos and .bhv files. 
#-------------------------------------------------------
# interactive nsplug only when not being remotely launched.
NSFLAGS="-s -f"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="-i -f"
fi

nsplug meta_vehicle_cn.moos targ_$VNAME.moos $NSFLAGS WARP=$TIME_WARP \
       PSHARE_PORT=$PSHARE_PORT     VNAME=$VNAME                 \
       COLOR=$COLOR                 MAX_SPD=$MAX_SPD               \
       START_POS=$START_POS         SHORE_IP=$SHORE_IP           \
       SHORE_PSHARE=$SHORE_PSHARE   MOOS_PORT=$MOOS_PORT         \
       IP_ADDR=$IP_ADDR             START_POS=$START_POS         
       
nsplug meta_vehicle_cn.bhv targ_$VNAME.bhv $NSFLAGS VNAME=$VNAME \
       LOITER_SPD=$LOITER_SPD     COLOR=$COLOR                   \
       TRANSIT_SPD=$LOITER_SPD
       
if [ "${JUST_MAKE}" = "yes" ]; then
    vecho "Files assembled; nothing launched; exiting per request."
    exit 0
fi

#-------------------------------------------------------
#  Part 6: Launch the vehicle mission
#-------------------------------------------------------
vecho "Launching $VNAME MOOS Community. WARP="$TIME_WARP
pAntler targ_$VNAME.moos >& /dev/null &
vecho "Done Launching $VNAME MOOS Community"

#---------------------------------------------------------------
#  Part 7: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#---------------------------------------------------------------
# Part 8: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_$VNAME.moos
kill -- -$$
