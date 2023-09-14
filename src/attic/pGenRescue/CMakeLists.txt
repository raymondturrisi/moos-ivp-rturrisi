# --------------------------------------------------------
# The CMakeLists.txt for:                       pGenPath Author(s):
# --------------------------------------------------------

set(SRC GenRescue.cpp GenRescue_Info.cpp main.cpp)

add_executable(pGenRescue ${SRC})

target_link_libraries(
  pGenRescue
  ${MOOS_LIBRARIES}
  geometry
  apputil
  mbutil
  m
  pthread)
