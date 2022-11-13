cmake_minimum_required(VERSION 3.0.0)
project(LEO VERSION 1.0.0)
set (CMAKE_CXX_STANDARD 17)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})

set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)



include_directories("include/")
include_directories("lib/")
set(EIGEN_DIR "lib/eigen-3.4.0")
include_directories(${EIGEN_DIR})

add_executable(LEO src/gnc/attitude_controller.cpp)