#On Ubuntu 18.04, get the latest CMake from https://apt.kitware.com/.
cmake_minimum_required(VERSION 3.18)

project(plane_segment LANGUAGES C CXX)

# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Find installed Open3D, which exports Open3D::Open3D
find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)

set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3/")
include_directories(${EIGEN_INCLUDE_DIR})
include_directories("./src/")

add_executable(plane_segment)
target_sources(plane_segment PRIVATE src/main.cpp
	 src/boxPlaneSegment.cpp)
target_link_libraries(plane_segment PRIVATE Open3D::Open3D)
