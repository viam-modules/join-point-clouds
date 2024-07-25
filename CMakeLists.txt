cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(join-point-clouds
    DESCRIPTION "Viam Module to Join Point Clouds"
    HOMEPAGE_URL https://github.com/hexbabe/join-point-clouds
    LANGUAGES CXX
)
set(CMAKE_CXX_STANDARD 17)
enable_language(C)  # required for MPI

find_package(MPI REQUIRED)
find_package(viam-cpp-sdk REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io kdtree registration)

# Explicitly set the PCL directory paths
set(PCL_DIR /usr/lib/aarch64-linux-gnu/cmake/pcl)
include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(join-point-clouds src/main.cpp src/utils.cpp)

target_link_libraries(join-point-clouds PUBLIC viam-cpp-sdk::viamsdk ${CMAKE_DL_LIBS} ${PCL_LIBRARIES})

target_compile_options(join-point-clouds PRIVATE
    -O2
    -Wredundant-move 
    -Wpessimizing-move 
)
