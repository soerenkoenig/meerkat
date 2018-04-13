cmake_minimum_required(VERSION 3.10)

project(meerkat VERSION 0.0.1 LANGUAGES CXX )


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_LIBRARIES )
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/build")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/build/")
# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)
link_libraries("-stdlib=libc++")
# Include Boost as an imported target
find_package(Boost REQUIRED)
add_library(boost INTERFACE IMPORTED)
set_property(TARGET boost PROPERTY
        INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})


# Targets that we develop here
enable_testing()
add_subdirectory(libowl)
#add_subdirectory(app)



