#!/bin/bash
#Example directory conversion script

target_directory="$1"
destination_directory="${1}_tmp"

python3 ../../MWDataMgr/MWDataMgr.py -d $target_directory/ \
    -o $destination_directory \
    -i mwmgr/mw_ix.cfg \
    -t csv \
    --moos \
    --topic_mapping mwmgr/mw_moos_topic_mapping.cfg 