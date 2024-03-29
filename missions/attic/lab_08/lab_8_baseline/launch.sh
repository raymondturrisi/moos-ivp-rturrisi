#!/bin/bash 
#-------------------------------------------------------
#   Script: launch_all.sh                       
#  Mission: lab_10_baseline
#-------------------------------------------------------
#  Part 1: Set global var defaults
#----------------------------------------------------------
TIME_WARP=1
JUST_MAKE=""

#-------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "launch_all.sh [SWITCHES] [WARP]               " 
	echo "  --help, -h                                  " 
	echo "    Display this help message                 "
	echo "  --just_make, -j                             " 
	echo "    Just make targ files, but do not launch   "
	echo "                                              "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="-j"
    else 
	echo "launch_all.sh: Bad Arg:" $ARGI "Exit Code 1."
	exit 1
    fi
done

#-------------------------------------------------------
#  Part 3: Launch the vehicles
#-------------------------------------------------------
echo "Launching henry...."
./launch_vehicle.sh --vname=henry --mport=9001 --pshare=9201  \
		    --nogui --nc --auto $JUST_MAKE $TIME_WARP &

echo "Launching gilda...."
./launch_vehicle.sh --vname=gilda --mport=9002 --pshare=9202  \
		    --nogui --nc --auto $JUST_MAKE $TIME_WARP &

#-------------------------------------------------------
#  Part 4: Launch the shoreside
#-------------------------------------------------------
echo "Launching shoreside...."
./launch_shoreside.sh $JUST_MAKE $TIME_WARP --ip=localhost --nc


