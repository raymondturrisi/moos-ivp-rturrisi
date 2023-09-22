#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch.sh     
#  Mission: convoy_ctx
#   Author: Michael Benjamin   
#   LastEd: November 2021
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#-------------------------------------------------------------- 
#  Part 2: Set Global variables
#-------------------------------------------------------------- 
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE=""
MISSION="convoy_mit"
VERBOSE=""
AUTO_LAUNCHED="no"
CMD_ARGS=""
NOGUI=""
NOCONFIRM="-nc"

RANDSTART="true"
SHORE_PSHARE="9200"
SHOREIP="localhost"
AMT=4
CNAMT=0
MEDIATED=""

TRANSIT_SPD=0.5
MAXIMUM_SPD=2.0

CONVOY_VERS=""

MTASC="no"
MTASC_SUBNET="192.168.7"
USE_CACHE=""
VLAUNCH_ARGS=" --auto "
SLAUNCH_ARGS=" --auto "

#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                               "
	echo "                                                        "
        echo "Synopsis:                                               "
	echo "  A script for launching an arbitrary number of vehicles"
	echo "                                                        "
	echo "Options:                                                "
        echo "  --help, -h                                            "
        echo "    Display this help message                           "
        echo "  --verbose, -v                                         "
        echo "    Increase verbosity                                  "
	echo "  --just_make, -j                                       " 
	echo "    Just make the targ files, but do not launch.        " 
	echo "  --amt=N                                               " 
	echo "    The number of vehicles to launch. The default is 1. "
	echo "  --cn=N                                                " 
	echo "    The number of contacts to launch. The default is 0. "
	echo "  --active_convoy, -ac                                  " 
	echo "    Use Active Convoy Behavior: BHV_ConvoyV21X          "
	echo "  --nomediate, -nm                                      " 
	echo "    No use of pMediator for inter-vessel comms          "
	echo "  --norand                                              " 
	echo "    Do not randomly generate files vpositions.txt. Just "
	echo "    re-use the previous versions if they exist.         "
	echo "                                                        "
	echo "Options For launches on MTASC machines:                 "
	echo "                                                        "
	echo "  --mtasc, -m                                           " 
	echo "    Launch vehicles in the MTASC cluster                " 
	echo "  --shoreip=<ipaddr>                                    " 
	echo "    IP address where nodes can expect to find shoreside "
	echo "  --cache, -c                                           " 
	echo "    Dont confirm IP address of a pablo before launching "
	echo "    a mission. Instead, rely on the ~/.pablos_ipfs file "
	echo "    for a cached mapping of pablo names to IP addresses."
	echo "    Without this flag, the pablo name/IP will first be  "
	echo "    confirmed by contacting the pablo. Longer to launch."
	echo "  --logclean, -l                                        " 
        echo "    Clean (remove) all log files prior to launch        "
	echo "                                                        "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE="--verbose"
	NOCONFIRM=""
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	VLAUNCH_ARGS+=" $ARGI"
	SLAUNCH_ARGS+=" $ARGI"
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--logclean" -o "${ARGI}" = "-l" ]; then
	VLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI}" = "--norand" -o "${ARGI}" = "-r" ]; then
	RANDSTART="false"
    elif [ "${ARGI}" = "--mtasc" -o "${ARGI}" = "-m" ]; then
	MTASC="yes"
    elif [ "${ARGI}" = "--nomediate" -o "${ARGI}" = "-nm" ]; then
	MEDIATED="--nomediate"
    elif [ "${ARGI:0:10}" = "--shoreip=" ]; then
        SHOREIP="${ARGI#--shoreip=*}"
    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHOREIP="${ARGI#--shoreip=*}"
    elif [ "${ARGI:0:9}" = "--maxspd=" ]; then
	VLAUNCH_ARGS=" $ARGI"
    elif [ "${ARGI}" = "--cache" -o "${ARGI}" = "-c" ]; then
        USE_CACHE="--cache"
    elif [ "${ARGI}" = "--active_convoy" -o "${ARGI}" = "-ac" ]; then
	CONVOY_VERS="--active_convoy"
    elif [ "${ARGI:0:6}" = "--amt=" ]; then
        AMT="${ARGI#--amt=*}"
	if [ ! $AMT -ge 1 ]; then
	    echo "$ME: Vehicle amount must be >= 1."
	    exit 1
	fi
    elif [ "${ARGI:0:5}" = "--cn=" ]; then
        CNAMT="${ARGI#--cn=*}"
	if [ ! $CNAMT -ge 1 ]; then
	    echo "$ME: contact amount must be >= 0."
	    exit 1
	fi
    else 
	echo "$ME: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

#---------------------------------------------------------------
#  Part 4: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" != "" ]; then 
    echo "======================================================"
    echo "              launch.sh SUMMARY                       "
    echo "======================================================"
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]      "
    echo "TIME_WARP =     [${TIME_WARP}]     "
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}] "
    echo "AMT =           [${AMT}]           "
    echo "CNAMT =         [${CNAMT}]         "
    echo "MEDIATED =      [${MEDIATED}]      "
    echo "---------------------------------- "
    echo "TRANSIT_SPD =   [${TRANSIT_SPD}]   "
    echo "MAXIMUM_SPD =   [${MAXIMUM_SPD}]   "
    echo "CONVOY_VERS =   [${CONVOY_VERS}]   "
    echo "---------------------------------- "
    echo "SHOREIP      =   [${SHOREIP}]      "
    echo "MTASC        =   [${MTASC}]        "
    echo "MTASC_SUBNET =   [${MTASC_SUBNET}] "
    echo -n "Hit the RETURN key to continue with launching"
    read ANSWER
fi

#-------------------------------------------------------------
# Part 5: Generate random starting positions, speeds and vnames
#-------------------------------------------------------------
vecho "Picking Convoy vehicle starting positions."

if [ ! -f "$HOME/.pablo_names" ]; then
    if  [ "$MTASC" = "yes" ]; then
	echo "$ME: Could not find ~/.pablo_names. Exit Code 2."
	exit 2
    fi
fi

if [ "${RANDSTART}" = "true" -o  ! -f "vpositions.txt" ]; then
    ./pickpos.sh $AMT
fi

# vehicle names and colors are always deterministic
pickpos --amt=$AMT --vnames=abe,ben,cal,deb,eve,fin,ned,max,oak  > vnames.txt
pickpos --amt=$AMT --colors  > vcolors.txt

VEHPOS=(`cat vpositions.txt`)
VNAMES=(`cat vnames.txt`)
COLORS=(`cat vcolors.txt`)
PABLOS=(`cat ~/.pablo_names`)  # only need in mtasc mode

#-------------------------------------------------------------
# Part 6: In MTASC mode, the shore IP address must be: 
#         (1) a non-localhost IP address. 
#         (2) a currently active IP interface for this machine
#         (3) an IP address on the local MTASC network
#-------------------------------------------------------------
if [ "$MTASC" = "yes" ]; then
    vecho "Verifying Shoreside IP Address for MTASC mode"

    ADDR=""
    if [ "$SHOREIP" = "localhost" ]; then
        ADDR=`ipmatch.sh --match=$MTASC_SUBNET`
    else
	ADDR=`ipmatch.sh --match=$SHOREIP`
    fi
    
    if [ "$ADDR" != "" ]; then
	SHOREIP="$ADDR"
    else
	echo "Provide active Shore IP addr on MTASC network. Exit Code 3."
	exit 3
    fi

    # verify_ssh_key.sh --pablo
fi

#-------------------------------------------------------------
# Part 7: Launch the convoy vehicles
#-------------------------------------------------------------
ALL_VNAMES=""
for INDEX in `seq 1 $AMT`;
do
    sleep 0.2
    ARRAY_INDEX=`expr $INDEX - 1`
    START=${VEHPOS[$ARRAY_INDEX]}
    VNAME=${VNAMES[$ARRAY_INDEX]}
    COLOR=${COLORS[$ARRAY_INDEX]}

    if [ "${ALL_VNAMES}" != "" ]; then
	ALL_VNAMES+=":"
    fi
    ALL_VNAMES+=$VNAME
    
    VLAUNCH_ARGS+=" $NOCONFIRM "
    IX_VLAUNCH_ARGS=$VLAUNCH_ARGS
    IX_VLAUNCH_ARGS+=" --index=$INDEX --start=$START    "
    IX_VLAUNCH_ARGS+=" --maxspd=$MAXIMUM_SPD            "
    IX_VLAUNCH_ARGS+=" --speed=$TRANSIT_SPD             " 
    IX_VLAUNCH_ARGS+=" --color=$COLOR  --sim            " 
    IX_VLAUNCH_ARGS+=" --vname=$VNAME  --shore=$SHOREIP "
    IX_VLAUNCH_ARGS+=" $VERBOSE $CONVOY_VERS $MEDIATED  "
    IX_VLAUNCH_ARGS+=" $TIME_WARP "

    #vecho "Launching: $VNAME"
    if [ "${MTASC}" != "yes" ]; then
	./launch_vehicle.sh $IX_VLAUNCH_ARGS	
    else
	SSH_OPTIONS="-o StrictHostKeyChecking=no -o LogLevel=QUIET "
	SSH_OPTIONS+="-o ConnectTimeout=1 -o ConnectionAttempts=1 "
	SSH_OPTIONS+="-o BatchMode=yes "

	PNAME=${PABLOS[$ARRAY_INDEX]}
	IP=`find_pablo_ip_by_name.sh $USE_CACHE $PNAME | tr -d '\r' `
    
	vecho "pablo[$PNAME] IP is: $IP"
	IX_VLAUNCH_ARGS+=" --mission=$MISSION --mtasc --ip=$IP"
	ssh student2680@$IP $SSH_OPTIONS mlaunch.sh $IX_VLAUNCH_ARGS &
    fi
done

#-------------------------------------------------------------
# Part 7: Launch the Shoreside mission file
#-------------------------------------------------------------
vecho "Launching the shoreside. Args: $SLAUNCH_ARGS $TIME_WARP"

SLAUNCH_ARGS+=" --vnames=$ALL_VNAMES "

./launch_shoreside.sh $SLAUNCH_ARGS $VERBOSE $TIME_WARP
sleep 0.25

#---------------------------------------------------------------
#  Part 8: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" -o "${JUST_MAKE}" != "" ]; then
    exit 0
fi

#-------------------------------------------------------------
# Part 9: Launch uMAC in paused mode until the mission is quit
#-------------------------------------------------------------
uMAC --paused targ_shoreside.moos
kill -- -$$
