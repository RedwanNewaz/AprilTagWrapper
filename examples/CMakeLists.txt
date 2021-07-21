
# add dependencies
include(../cmake/CPM.cmake)

CPMAddPackage("gh:jbeder/yaml-cpp#yaml-cpp-0.6.3@0.6.3")

add_executable(example_april_tag_ekf example_april_tag_ekf.cpp
        april_tag_wrapper/april_tag_wrapper.cpp)

find_package(OpenCV QUIET)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(example_april_tag_ekf
        kalman_filters
        apriltag
        yaml-cpp
        ${OpenCV_LIBS}
        )

# python bindings
find_package(PythonLibs 3.7 REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)
add_library(apriltagWrapper example_python_apriltag_ekf.cpp)
target_link_libraries(apriltagWrapper apriltag ${Boost_LIBRARIES}
        ${PYTHON_LIBRARIES}
        Eigen3::Eigen
        )
target_include_directories(apriltagWrapper PRIVATE ${PYTHON_INCLUDE_DIRS})