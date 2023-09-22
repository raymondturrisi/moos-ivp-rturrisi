

# Strip variables worth ignoring from a mission

newest_mission=(logs/$1*)
newest_mission=${newest_mission[0]}

rm -rf $newest_mission/**/**.blog $newest_mission/**/**.ylog

deletions="\
    DB_EVENT \
    DB_CLIENTS \
    LOGGER_DIRECTORY \
    APPCAST \
    PSHARE_INPUT_SUMMARY \
    PSHARE_OUTPUT_SUMMARY \
    APPCAST_REQ \
    UMH_SUMMARY_MSGS \
    NODE_PSHARE_VARS \
    APP_LOG \
    PROC_WATCH_TIME_WARP \
    PROC_WATCH_EVENT \
    PROC_WATCH_ALL_OK \
    PROC_WATCH_SUMMARY \
    PROC_WATCH_FULL_SUMMARY \
    HELM_MAP_CLEAR \
    PNODEREPORTER_PID \
    UFLDNODEBROKER_PID \
    PSHARE_CMD \
    UMH_SUMMARY_MSGS \
    NODE_BROKER_ACK \
    NODE_PSHARE_VARS \
    MISSION_HASH \
    HELM_MAP_CLEAR \
    REALMCAST_CHANNELS \
    IVPHELM_REGISTER \
    CONTACTS_RECAP \
    CONTACT_RANGES \
    PCONTACTMGRV20_PID \
    IVPHELM_CPU \
    PNR_EXTRAP_POS_GAP \
    PNR_EXTRAP_HDG_GAP \
    BHV_IPF \
    OPR_SECS_IN_POLY \
    OPR_DEBUG \
    OPR_TRAJECTORY_PERIM_DIST
    "

for DIR in $newest_mission/*; do
    for alog in $DIR/*.alog; do 
        mv $alog ${alog}-old
        alogrm -f -q ${alog}-old $deletions $alog
        rm ${alog}-old
    done 
    
done

# Compress the final results

7z a -t7z -m0=lzma -mx=9 -mfb=64 -md=32m -ms=on $newest_mission.7z $newest_mission &> /dev/null

rm -rf $newest_mission