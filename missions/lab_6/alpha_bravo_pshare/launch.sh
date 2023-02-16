#!/bin/bash -e 

#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------

TIME_WARP=1
JUST_MAKE="no"
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	printf "%s [SWITCHES] [time_warp]   \n" $0
	printf "  --just_make, -j    \n" 
	printf "  --help, -h         \n" 
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ] ; then
	JUST_MAKE="yes"
    else 
        echo "launch.sh Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done



#----------------------------------------------------------
 #  Part 3: Launch the processes 
 #---------------------------------------------------------- 
 echo "Launching All MOOS Communities with WARP:" $TIME_WARP

 pAntler shoreside.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &
 sleep 1
 pAntler alpha.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &
 sleep 1
 pAntler bravo.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &

 uMAC -t shoreside.moos