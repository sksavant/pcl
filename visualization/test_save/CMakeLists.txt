cmake_minimum_required(VERSION 2.8)
project(test_hist_save)

find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

include_directories(${VTK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories("/media/DATA/home/repos/gsoc_repos/pcl/io/src")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_executable(test_save test.cpp /media/DATA/home/repos/gsoc_repos/pcl/io/src/pcd_io.cpp)
target_link_libraries(test_save ${PCL_LIBRARIES} ${VTK_LIBRARIES})
