cmake_minimum_required(VERSION 3.10)
project(GenericFilters)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

# make library
set(KF_SOURCE_FILES
        KF/ekf.cpp
        KF/ukf.cpp
        KF/filter_base.cpp
        KF/filter_utilities.cpp
        KF/ekf_interface.cpp )

set(KF_HEADER_FILES
        include/robot_localization/ekf.h
        include/robot_localization/filter_base.h
        include/robot_localization/filter_common.h
        include/robot_localization/filter_utilities.h
        include/robot_localization/ukf.h
        include/robot_localization/ekf_interface.h
        )

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_library(kalman_filters ${KF_SOURCE_FILES} ${KF_HEADER_FILES})

target_link_libraries(kalman_filters Eigen3::Eigen)

# add april tag for testing kalman filters
add_subdirectory(apriltag)
include_directories(apriltag)
add_subdirectory(examples)









# TODO install kalman filter libraries

