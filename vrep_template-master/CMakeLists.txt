cmake_minimum_required(VERSION 3.12)

project(Mycar)

#Make sure no in-source build -----------------------------------------------------------------------------#
if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

#Configure CMake and Compiler -----------------------------------------------------------------------------#
set( CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set( CMAKE_CXX_STANDARD 14)
set( CMAKE_C_STANDARD 11)
if(UNIX)
  set( CMAKE_CXX_FLAGS "-pthread")
endif()
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin) 
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib) 

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message("Setting build type to 'Debug' as none was specified.")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for test_footStest_footPlacementtepPlan-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

add_definitions(-DLOCAL_SIMULATION)
add_definitions(-DNON_MATLAB_PARSING
                -DMAX_EXT_API_CONNECTIONS=255
                -DWITHOUT_NUMPY
                -DDO_NOT_USE_SHARED_MEMORY)

#Add sub directories --------------------------------------------------------------------------------------#
add_subdirectory(core_test)
add_subdirectory(coppeliaSim-client)

#Add libs and include_directories -------------------------------------------------------------------------#
include_directories(".")
include_directories("core_test")
include_directories("coppeliaSim-client")
include_directories("coppeliaSim-client/include")
include_directories("coppeliaSim-client/include/stack")
include_directories("coppeliaSim-client/remoteApi")

file(GLOB USER_SOURCE "*.cpp")
add_executable(${PROJECT_NAME} ${USER_SOURCE})

target_link_libraries(${PROJECT_NAME} core_test)
target_link_libraries(${PROJECT_NAME} coppeliaSim_client)

