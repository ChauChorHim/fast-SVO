cmake_minimum_required(VERSION 2.8)
project(fast_SVO)


find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 is required.")
    endif()
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(
    ${PROJECT_NAME} SHARED
    src/Dataset.cc
    src/System.cc
    src/Tracking.cc
    src/ORBextractor.cc
)



# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(
    stereo_kitti
    Examples/Stereo/stereo_kitti.cc
)
target_link_libraries(
    stereo_kitti
    ${OpenCV_LIBS}
)
target_link_libraries(stereo_kitti ${PROJECT_NAME})


# Test
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

add_executable(
    test_Fast
    test/test_Fast.cpp
)
target_link_libraries(
    test_Fast
    ${OpenCV_LIBS}
)

add_executable(
    test_ORB
    test/test_ORB.cpp
)
target_link_libraries(
    test_ORB
    ${OpenCV_LIBS}
)