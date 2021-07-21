cmake_minimum_required(VERSION 3.10)
project(GenericFilters)

set(CMAKE_CXX_STANDARD 17)

# add dependencies
include(cmake/CPM.cmake)

include_directories(include
        apriltag
        )

# make library
set(KF_SOURCE_FILES
        KF/ekf.cpp
        KF/ukf.cpp
        KF/filter_base.cpp
        KF/filter_utilities.cpp
        KF/filter_interface.cpp include/robot_localization/filter_interface.h)
set(KF_HEADER_FILES
        include/robot_localization/ekf.h
        include/robot_localization/filter_base.h
        include/robot_localization/filter_common.h
        include/robot_localization/filter_utilities.h
        include/robot_localization/ukf.h
        )

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_library(kalman_filters ${KF_SOURCE_FILES} ${KF_HEADER_FILES})
target_link_libraries(kalman_filters Eigen3::Eigen)


# add april tag for testing kalman filters
add_subdirectory(apriltag)


add_executable(GenericFilters main.cpp
        april_tag_wrapper/april_tag_wrapper.cpp)


find_package(OpenCV QUIET)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        kalman_filters
        apriltag
        ${OpenCV_LIBS}
        )

# TODO install kalman filter libraries

