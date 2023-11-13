#--------------------------------------------------------
# The CMakeLists.txt for:             lib_behavior_convoy
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

#include_directories(${CMAKE_SOURCE_DIR}/../moos-ivp-pavlab/src/lib_convoy)

#MACRO(ADD_BHV BHV_NAME)
#  ADD_LIBRARY(${BHV_NAME} SHARED ${BHV_NAME}.cpp)
#  TARGET_LINK_LIBRARIES(${BHV_NAME} 
#    convoyz
#    helmivp
#    behaviors 
#    ivpbuild 
#    logic 
#    ivpcore 
#    bhvutil 
#    mbutil 
#    geometry 
#    ${SYSTEM_LIBS} )
#ENDMACRO(ADD_BHV)

#--------------------------------------------------------
#                                          BHV_ConvoyV21X
#--------------------------------------------------------
ADD_LIBRARY(BHV_ConvoyV21Z SHARED 
  BHV_ConvoyV21Z.cpp
  ConvoyMarker.cpp
  MarkerTail.cpp
  )

TARGET_LINK_LIBRARIES(BHV_ConvoyV21Z
   helmivp
   convoyz
   behaviors
   contacts
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
   ${MOOS_LIBRARIES}
   ${SYSTEM_LIBS} )
