# ##############################################################################
# FILE:        moos-ivp-extend/src/CMakeLists.txt DATE:        2010/09/07
# 2020/05/09 minor mods DESCRIPTION: CMakeLists.txt file for the moos-ivp-extend
# source directory
# ##############################################################################

# ============================================================================
# Add the libraries in the current directory to the include path
# ============================================================================
file(GLOB LOCAL_LIBRARY_DIRS ./lib_*)
include_directories(${LOCAL_LIBRARY_DIRS})

# ============================================================================
# List the subdirectories to build...
# ============================================================================

# add_subdirectory(lib_behaviors-test) add_subdirectory(pExampleApp)
# add_subdirectory(pXRelayTest) add_subdirectory(pOdometry)
# add_subdirectory(pOdometryUUV) add_subdirectory(pPointAssign)
# add_subdirectory(pGenPath) add_subdirectory(pGenRescue)
add_subdirectory(app_pickposx)

# ##############################################################################
# END of CMakeLists.txt
# ##############################################################################
