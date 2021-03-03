cmake_minimum_required(VERSION 3.5)
project(atlas_ceres)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# fPIE is require because of the way we pass pointers between c++ and rust code
set(CMAKE_CXX_FLAGS "-fPIE -Wall -Wextra -std=c++20")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_C_COMPILER "/usr/bin/clang") #using clang because rust uses clang
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

find_package(Ceres REQUIRED)

file(GLOB_RECURSE SOURCE_CODE
    ../src/bundle_adjustment.cpp
    ../src/utils.cpp
    ../src/ceres_options.cpp
    ../src/read_write.cpp
    )


add_executable(create_ba_data create_bundle_adjustment_data.cpp ${SOURCE_CODE})
target_link_libraries(create_ba_data Ceres::ceres)

add_executable(test_ba test_bundle_adjustment.cpp ${SOURCE_CODE})
target_link_libraries(test_ba Ceres::ceres)
