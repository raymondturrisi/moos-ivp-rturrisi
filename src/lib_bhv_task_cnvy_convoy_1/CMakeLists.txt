#--------------------------------------------------------
# The CMakeLists.txt for:             lib_behaviors-swarm
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
#                                          BHV_TaskConvoy
#--------------------------------------------------------
ADD_LIBRARY(BHV_TaskConvoy1 SHARED 
  BHV_TaskConvoy1.cpp )

target_include_directories(BHV_TaskConvoy1 PRIVATE ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/src/lib_helmtask)

find_library(HELMTASK_LIBRARY NAMES helmtask PATHS ${CMAKE_SOURCE_DIR}/../moos-ivp-swarm/lib NO_DEFAULT_PATH)

if(NOT HELMTASK_LIBRARY)
  #TODO: Can make it so this app doesn't build?
  message(FATAL_ERROR "helmtask library not found")
endif()

TARGET_LINK_LIBRARIES(BHV_TaskConvoy1
  ${HELMTASK_LIBRARY}
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

