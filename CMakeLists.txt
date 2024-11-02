cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-std=c++17")
project(robot VERSION 1.0)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)


add_definitions(-DDEBUG)

# find_package(absl REQUIRED)

# find_package(glog)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Eigen3)
find_package(Sophus REQUIRED)

# find_package(absl)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(Robot SHARED
    include/robotics/body.h
    include/robotics/robot.h
    include/robotics/joint.h
    include/robotics/mathfunction.h
    include/robotics/endeffector.h
    src/robotics/mathfunction.cpp
    src/robotics/joint.cpp
    src/robotics/body.cpp
    src/robotics/robot.cpp
    src/robotics/endeffector.cpp
)

target_link_libraries(Robot Sophus::Sophus)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)



add_executable(optimization src/optimization/optimization.cpp)

add_executable(optimizationSophus src/optimization/optimizationSophus.cpp)
target_link_libraries(optimizationSophus Sophus::Sophus)

add_executable(helloworld src/optimization/optimizationCeres.cpp)
target_link_libraries(helloworld ${CERES_LIBRARIES})

add_executable(testFourBarLinkageFK src/main_testFourBarLinkabgeFK.cpp)
target_link_libraries(testFourBarLinkageFK Robot)

add_executable(testFourBarLinkageIK src/main_testFourBarLinkabgeIK.cpp)
target_link_libraries(testFourBarLinkageIK Robot)

