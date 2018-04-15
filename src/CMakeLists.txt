cmake_minimum_required(VERSION 3.10)

project(meerkat VERSION 0.0.1 LANGUAGES CXX )



set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_LIBRARIES )
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/build")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/build/")
# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
# using Visual Studio C++
# Disable MSVC warnings that suggest making code non-portable.
set(CMAKE_CXX_FLAGS "/wd4996 ${CMAKE_CXX_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++latest")
else()
    set(CMAKE_CXX_STANDARD 17)
endif()
# Include Boost as an imported target
#find_package(Boost REQUIRED)
#add_library(boost INTERFACE IMPORTED)
#set_property(TARGET boost PROPERTY
#        INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})


# Targets that we develop here
enable_testing()
add_subdirectory(libowl)
#add_subdirectory(app)



