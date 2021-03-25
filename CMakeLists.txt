cmake_minimum_required(VERSION 3.16)
project(myslam)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

# CXX FLAGS: Compilation flags for cxx files, 
# -Wall: turn on warning
# -std: specify the c++ version
# -O3: high level of optimaztion, slower compile time
# -fopenmp & -pthread: enable OpenMP for multi-threading, (more than including library)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall") 
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 -fopenmp -pthread")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


#################### dependencies

set(EIGEN_INCLUDE_DIRS "/usr/include/eigen3/")
include_directories(EIGEN_INCLUDE_DIRS)

find_package(OpenCV 4.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

find_package(Glog REQUIRED) # glog, for logging
include_directories(${Glog_INCLUDE_DIRS})

find_package(GTest REQUIRED)
include_directories(${GTest_INCLUDE_DIRS})

find_package(GFlags REQUIRED)
include_directories(${GFlags_INCLUDE_DIRS})

find_package(CSparse REQUIRED)
include_directories(${CSparse_INCLUDE_DIRS})

set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${Sophus_LIBRARIES}
    GL GLU GLEW glut
    g2o_core g2o_stuff g2o_types_sba g20_solver_csparse g2o_csparse_extension
    ${GTEST_BOTH_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    pthread
    ${CSPARSE_LIBRARIES})

# enable_testing()

################## source and test
include_directories("${PROJECT_SOURCE_DIR}/include")
add_subdirectory(src)
