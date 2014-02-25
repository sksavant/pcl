cmake_minimum_required(VERSION 2.8)
project(test_hist_save)

find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

include_directories(${VTK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(test_save test.cpp)
target_link_libraries(test_save ${PCL_LIBRARIES} ${VTK_LIBRARIES})
