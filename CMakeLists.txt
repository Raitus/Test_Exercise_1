cmake_minimum_required(VERSION 3.21)
project(Test_Exercise_1)

set(CMAKE_CXX_STANDARD 17)

add_executable(Test_Exercise_1 main.cpp)
set(VTK_DIR "/home/raitus/vtk/sources_8.2/VTK-8.2.0/Build/ReleaseGCC")

find_package(PCL 1.12 REQUIRED)
#HINTS  #/lib/x86_64-linux-gnu/cmake/pcl /usr/include/pcl-1.10
#find_package(PCL 1.10 COMPONENTS common io visualization)

if( PCL_FOUND )
    include_directories(${PCL_INCLUDE_DIRS}) #set to the paths to PCL installed headers and the dependency headers
    add_definitions(${PCL_DEFINITIONS}) #lists the needed preprocessor definitions and compiler flags
    link_directories(${PCL_LIBRARY_DIRS}) #set to the paths to where PCL libraries and 3rd party dependencies reside
    target_link_libraries(Test_Exercise_1
            ${PCL_LIBRARIES}
            pcl_visualization) #set to the file names of the built and installed PCL libraries
endif()