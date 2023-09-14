#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: pickpos_cn.sh
#   Author: Michael Benjamin   
#     Date: Nov 2nd 2021       
#    About: Pick positions for contact vehicles
#-------------------------------------------------------------- 

AMT=1
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	echo "pickpos.sh [AMOUNT]           " 
	echo "  --help, -h                       " 
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$AMT" = 1 ]; then 
        AMT=$ARGI
    else 
	echo "Bad Argument: $ARGI"
	exit 0
    fi
done

if [ ! $AMT -ge 0 ] ; then
    echo "Vehicle amount must be >= 0. Exiting now."
    exit 1
fi


#pickpos --poly="-110,350:-110,150:400,150:400,350" --buffer=20 \

pickpos --poly="-1400,400:-1400,600:-1100,600:-1100,400" \
	--poly="-1400,-600:-1400,-800:-1100,-800:-1100,-600" \
	--buffer=20 --amt=$AMT --hdg=0:359  > vpositions_cn.txt  

pickpos --amt=$AMT --vnames --reverse_names > vnames_cn.txt
