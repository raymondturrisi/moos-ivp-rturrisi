#--------------------------------------------------------
# The CMakeLists.txt for:           lib_bhv_task_waypoint
# Author(s):                                Mike Benjamin     
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
      )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m )
endif (${WIN32})

#--------------------------------------------------------
#                                        BHV_TaskWaypoint
#--------------------------------------------------------
ADD_LIBRARY(BHV_TaskWaypoint SHARED 
   BHV_TaskWaypoint.cpp )
TARGET_LINK_LIBRARIES(BHV_TaskWaypoint
   helmtask
   ufield
   helmivp
   contacts
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
   ${SYSTEM_LIBS} )

