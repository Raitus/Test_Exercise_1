cmake_minimum_required(VERSION 3.21)
project(Test_Exercise_1)

set(CMAKE_CXX_STANDARD 17)

add_executable(Test_Exercise_1 main.cpp)

# Find VTK
set(VTK_DIR "/home/raitus/vtk/sources_8.2/VTK-8.2.0/Build/ReleaseGCC") #path to VTK 8.2 config
#set(VTK_DIR "/home/raitus/ProgrammingLibraries/VTK/VTK_master/vtk_master_build") #path to VTK 9.1 config
find_package(VTK REQUIRED)

# Find PCL
find_package(PCL 1.11 COMPONENTS common io visualization)

if (PCL_FOUND AND VTK_FOUND)
    include_directories(${PCL_INCLUDE_DIRS}) #set to the paths to PCL installed headers and the dependency headers

    add_definitions(${PCL_DEFINITIONS}) #lists the needed preprocessor definitions and compiler flags

    link_directories(${PCL_LIBRARY_DIRS}) #set to the paths to where PCL libraries and 3rd party dependencies reside

    target_link_libraries(Test_Exercise_1
            ${PCL_LIBRARIES}
            pcl_visualization) #set to the file names of the built and installed PCL libraries
    target_link_libraries(Test_Exercise_1 ${VTK_LIBRARIES})
endif()