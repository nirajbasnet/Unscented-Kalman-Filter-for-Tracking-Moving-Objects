project(UKF_Tracking)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
   src/ukf.cpp
   src/main.cpp
   src/tools.cpp)

add_executable(UnscentedKF ${sources})
find_package(Python3 COMPONENTS Development NumPy)
target_include_directories(UnscentedKF PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
target_link_libraries(UnscentedKF Python3::Python Python3::NumPy)

