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

#IF( EXISTS ${CMAKE_SOURCE_DIR}/src/lib_eigen )
#  ADD_SUBDIRECTORY(lib_eigen)
#ENDIF()
add_subdirectory(app_test_eigen)
add_subdirectory(app_test_armadillo)
add_subdirectory(lib_bhv_task_cnvy_convoy_1)
add_subdirectory(lib_bhv_task_cnvy_convoy_2)
add_subdirectory(lib_bhv_task_cnvy_convoy_3) 
add_subdirectory(lib_convoyz)
add_subdirectory(lib_bhv_convoyz) 
add_subdirectory(lib_bhv_convoypd) 
#add_subdirectory(lib_bhv_task_cnvy_waypt_1)
#add_subdirectory(lib_bhv_task_cnvy_waypt_2)
add_subdirectory(lib_bhv_task_cnvy_waypt_3) 

# ##############################################################################
# END of CMakeLists.txt
# ##############################################################################
