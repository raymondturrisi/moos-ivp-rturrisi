#--------------------------------------------------------
# The CMakeLists.txt for:                       pHydrolink
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  Hydrolink.cpp
  Hydrolink_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pHydrolink ${SRC})

TARGET_LINK_LIBRARIES(pHydrolink
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

