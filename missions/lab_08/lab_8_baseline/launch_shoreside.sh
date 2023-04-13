#!/bin/bash 
#-------------------------------------------------------
#   Script: launch_shoreside.sh                       
#  Mission: lab_08_baseline
#-------------------------------------------------------
#  Part 1: Set global var defaults
#----------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
VERBOSE=""
CONFIRM="yes"
CMD_ARGS=""

IP_ADDR="localhost"
MOOS_PORT="9000"
PSHARE_PORT="9200"


VNAME1="gilda"  
VNAME2="henry"  
YMIN=-175
YMAX=-25
XMIN=-25
XMAX=200

#-------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    CMD_ARGS+="${ARGI} "
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [SWITCHES] [WARP]         "
	echo "  --help, -h                                  " 
	echo "    Display this help message                 "
	echo "  --just_make, -j                             " 
	echo "    Just make targ files, but do not launch   "
        echo "  --verbose, -v                               "
        echo "    Increase verbosity                        "
	echo "  --nc,-nc                                       " 
	echo "    No confirmation before launching             "
	echo "                                              "
	echo "  --ip=<localhost>                            " 
	echo "    Force pHostInfo to use this IP Address    "
	echo "  --mport=<9000>                              "
	echo "    Port number of this vehicle's MOOSDB port "
	echo "  --pshare=<9200>                             " 
	echo "    Port number of this vehicle's pShare port "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE="yes"
    elif [ "${ARGI}" = "--nc" -o "${ARGI}" = "-nc" ]; then
	CONFIRM="no"

    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:7}" = "--mport" ]; then
	MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"
    else
	echo "$ME Bad Arg:" $ARGI "Exit Code 1"
	exit 1
    fi
done


if [ "${VERBOSE}" = "yes" -o "${CONFIRM}" = "yes" ]; then 
    echo "$ME"
    echo "CMD_ARGS =    [${CMD_ARGS}]"
    echo "TIME_WARP =   [${TIME_WARP}]"
    echo "MOOS_PORT =   [${MOOS_PORT}]"
    echo "PSHARE_PORT = [${PSHARE_PORT}]"
    echo "IP_ADDR =     [${IP_ADDR}]"
    echo -n "Hit any key to continue with launching"
    read ANSWER
fi


#-------------------------------------------------------
#  Part 3: Create the .moos and .bhv files. 
#-------------------------------------------------------
nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP  \
       IP_ADDR=$IP_ADDR       PSHARE_PORT=$PSHARE_PORT             \
       MOOS_PORT=$MOOS_PORT  \
       XMIN=$XMIN XMAX=$XMAX YMIN=$YMIN YMAX=$YMAX \
       VNAME1=$VNAME1 VNAME2=$VNAME2

if [ ${JUST_MAKE} = "yes" ]; then
    echo "Files assembled; nothing launched; exiting per request."
    exit 0
fi

#-------------------------------------------------------
#  Part 4: Launch the processes
#-------------------------------------------------------
echo "Launching shoreside MOOS Community (WARP=%s) " $TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &

uMAC targ_shoreside.moos

#-------------------------------------------------------
#  Part 5: Exiting and/or killing the simulation
#-------------------------------------------------------
kill -- -$$
