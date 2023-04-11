cmake_minimum_required(VERSION 2.8.9)					                            # Minimum cmake requirement

project(gaze-control)							                            # ${PROJECT_NAME}

# cmake -DCMAKE_BUILD_TYPE=Release ..

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Debug)                                                                 # Release = Optimised
endif()

################################## Necessary packages to compile ###################################
find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)                                                                     # Links below to iDynTree
find_package(YARP 3.3.0 REQUIRED)                                                                   # Links below to ${YARP_LIBRARIES}

######################################## iCub related stuff ########################################
find_package(ICUBcontrib REQUIRED)                                                                  # Mandatory
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})                                           # Extend the current search path to load helpers
include(ICUBcontribHelpers)                                                                         # I don't know what this does
include(ICUBcontribOptions)                                                                         # I don't know what this does
icubcontrib_add_uninstall_target()					                            # Enables uninstall

#################################### Other files to be built #######################################
add_subdirectory(gazebo)                                                                            # Location of other CMakeLists
include_directories(include)                                                                        # Location of header files

#################################### Executables to be compiled ####################################
add_executable(gaze_control src/server include/RPCServerInterface.cpp include/GazeControl.cpp include/JointInterface.cpp include/QPSolver.cpp)
target_link_libraries(gaze_control Eigen3::Eigen iDynTree::idyntree-high-level ${YARP_LIBRARIES})

#################################### Executables to be compiled ####################################
add_executable(test src/test include/RPCServerInterface.cpp include/GazeControl.cpp include/JointInterface.cpp include/QPSolver.cpp)
target_link_libraries(test Eigen3::Eigen iDynTree::idyntree-high-level ${YARP_LIBRARIES})
