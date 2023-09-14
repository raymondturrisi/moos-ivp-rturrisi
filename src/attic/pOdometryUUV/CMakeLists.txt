#--------------------------------------------------------
# The CMakeLists.txt for:                    pOdometryUUV
# Author(s):                              Raymond Turrisi
#--------------------------------------------------------

SET(SRC
  OdometryUUV.cpp
  OdometryUUV_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pOdometryUUV ${SRC})

TARGET_LINK_LIBRARIES(pOdometryUUV
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

