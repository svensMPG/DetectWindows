cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

project(detectWindows)

find_package(PCL 1.8 REQUIRED)
find_package(libLAS REQUIRED)
find_package(vtk)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (detectWindows ./main.cpp ./src/pcl_las_helper.hpp ./src/miscFunctions.hpp ./src/CondEuclClustering.hpp ./src/hull2d.hpp
    ./h/extractfacade.h ./h/cloudsubtractor.h ./h/inrange.h ./h/areafromconcavehull.h ./h/removeconcavehullframe.h)
target_link_libraries (detectWindows ${PCL_LIBRARIES} ${libLAS_LIBRARIES} ${OpenCV_LIBS})
