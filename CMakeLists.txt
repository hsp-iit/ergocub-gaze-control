cmake_minimum_required(VERSION 3.16)					                            # Minimum cmake requirement

project(ergocub-gaze-control
	LANGUAGES CXX
    VERSION 0.0.100)

set(CMAKE_CXX_STANDARD 17)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Default build type to Release
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
  endif()

include(InstallBasicPackageFiles)
include(AddInstallRPATHSupport)
include(AddUninstallTarget)

################################## Necessary packages to compile ###################################
find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)                                                                     # Links below to iDynTree
find_package(YARP 3.3.0 REQUIRED)                                                                   # Links below to ${YARP_LIBRARIES}
find_package(eCubGazeControllerInterface REQUIRED)

#################################### Other files to be built #######################################
# add_subdirectory(gazebo)                                                                            # Location of other CMakeLists
include_directories(include)                                                                        # Location of header files

#################################### Executables to be compiled ####################################
add_executable(ergocub-gaze-control src/server.cpp include/GazeControl.cpp include/JointInterface.cpp include/QPSolver.cpp)
target_link_libraries(ergocub-gaze-control eCubGazeControllerInterface::eCubGazeControllerInterface Eigen3::Eigen iDynTree::idyntree-high-level ${YARP_LIBRARIES})

#################################### Executables to be compiled ####################################
add_executable(test src/test.cpp include/GazeControl.cpp include/JointInterface.cpp include/QPSolver.cpp)
target_link_libraries(test eCubGazeControllerInterface::eCubGazeControllerInterface Eigen3::Eigen iDynTree::idyntree-high-level ${YARP_LIBRARIES})

install(TARGETS ergocub-gaze-control DESTINATION bin) #test
