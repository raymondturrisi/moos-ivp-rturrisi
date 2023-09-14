# --------------------------------------------------------
# The CMakeLists.txt for:              lib_behaviors-test Author(s):
# --------------------------------------------------------

# Set System Specific Libraries
if(${WIN32})
  # Windows Libraries
  set(SYSTEM_LIBS)
else(${WIN32})
  # Linux and Apple Libraries
  set(SYSTEM_LIBS m)
endif(${WIN32})

macro(ADD_BHV BHV_NAME)
  add_library(${BHV_NAME} SHARED ${BHV_NAME}.cpp)
  target_link_libraries(
    ${BHV_NAME}
    helmivp
    behaviors
    ivpbuild
    logic
    ivpcore
    bhvutil
    mbutil
    geometry
    ${SYSTEM_LIBS})
endmacro(ADD_BHV)

# --------------------------------------------------------
# BHV_SimpleWaypoint
# --------------------------------------------------------
# ADD_LIBRARY(BHV_SimpleWaypoint SHARED BHV_SimpleWaypoint.cpp
# AOF_SimpleWaypoint.cpp) TARGET_LINK_LIBRARIES(BHV_SimpleWaypoint helmivp
# behaviors ivpbuild logic ivpcore bhvutil mbutil geometry ${SYSTEM_LIBS} )

# --------------------------------------------------------
# BHV_Pulse
# --------------------------------------------------------
add_library(BHV_Pulse SHARED BHV_Pulse.cpp)
target_link_libraries(
  BHV_Pulse
  helmivp
  behaviors
  ivpbuild
  logic
  ivpcore
  bhvutil
  mbutil
  geometry
  ${SYSTEM_LIBS})

# --------------------------------------------------------
# BHV_ZigLeg
# --------------------------------------------------------
add_library(BHV_ZigLeg SHARED BHV_ZigLeg.cpp)
target_link_libraries(
  BHV_ZigLeg
  helmivp
  behaviors
  ivpbuild
  logic
  ivpcore
  bhvutil
  mbutil
  geometry
  ${SYSTEM_LIBS})
