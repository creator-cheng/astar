cmake_minimum_required(VERSION 3.0.2)
project(astar)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
)


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)


include_directories(
    include 
    SYSTEM 
    # third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_executable(astar
    src/main.cpp
    src/astar.cpp
    )
target_link_libraries(astar
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)

