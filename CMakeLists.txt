cmake_minimum_required(VERSION 2.8)
project(fast_SVO)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb -Wno-int-in-bool-context")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-DCOMPILEDWITHC11)

find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 is required.")
    endif()
endif()

find_package(Eigen3 3.3.91 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(
    ${PROJECT_NAME} SHARED
    src/Dataset.cpp
    src/System.cpp
    src/Tracking.cpp
    src/Solver.cpp
    src/Timer.cpp
    src/Module.cpp
)



# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(
    stereo_kitti
    Examples/Stereo/stereo_kitti.cpp
)
target_link_libraries(
    stereo_kitti
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)
target_link_libraries(stereo_kitti ${PROJECT_NAME})


# Test
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

#add_executable(
#    test_P3P
#    test/testp3p.cpp
#)
#
#target_link_libraries(
#    test_P3P
#    ${EIGEN3_LIBS}
#)

#add_executable(
#    test_Timer
#    test/test_Timer.cpp
#)

add_executable(
    test_Module
    test/test_Module.cpp 
)