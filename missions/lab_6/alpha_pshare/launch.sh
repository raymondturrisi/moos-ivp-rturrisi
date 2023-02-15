#!/bin/bash -e 

pAntler shoreside.moos --MOOSTimeWarp=10 &
sleep 1
pAntler alpha.moos --MOOSTimeWarp=10 &

