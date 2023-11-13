#--------------------------------------------------------
# The CMakeLists.txt for:           lib_behavior_convoypd
# Author(s):                                  Ray Turrisi
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
#                                          BHV_ConvoyV21X
#--------------------------------------------------------
ADD_LIBRARY(BHV_ConvoyPD SHARED 
  BHV_ConvoyPD.cpp
  )

TARGET_LINK_LIBRARIES(BHV_ConvoyPD
   helmivp
   behaviors
   geometry
   contacts
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
   ${MOOS_LIBRARIES}
   ${SYSTEM_LIBS} )
