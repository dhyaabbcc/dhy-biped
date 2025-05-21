cmake_minimum_required(VERSION 3.5)
project(biped_control)
set(CMAKE_CXX_STANDARD 14)
include_directories(${CMAKE_BINARY_DIR})
set(CMAKE_C_FLAGS " -ggdb  -march=native -std=gnu99 -I.")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-lboost_system -fext-numeric-literals ") 

set(CMAKE_PREFIX_PATH "/opt/openrobots" ${CMAKE_PREFIX_PATH})

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(includePath)

add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)
find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    rospy
    gazebo_ros
    gazebo_msgs
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs  
)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)
find_package(pinocchio REQUIRED)
find_package(tsid REQUIRED)

#find_package(qpoases)
catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
)

set(QuadProgpp_DIR ${PROJECT_SOURCE_DIR}/third_party/QuadProgpp)
add_subdirectory(${QuadProgpp_DIR})
list(APPEND includePath "${QuadProgpp_DIR}/src")
# link_directories(${CMAKE_CURRENT_BINARY_DIR}/third_party/QuadProgpp)


include_directories(
    #include
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/wbc/inc"
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${includePath}
    #${GAZEBO_LIBRARY_DIRS}
)



link_directories(
    include/sdk/lib
    ${GAZEBO_LIBRARY_DIRS}
)

add_subdirectory(third_party/qpOASES)
# add_subdirectory(robotwrapper)

file(GLOB_RECURSE SRC_LIST
"src/*/*.cpp"
"src/*/*.h"
"ConvexMPC/*.cpp"
"robotwrapper/*.cpp"
"wbc/src/*.cpp"
)
add_executable(biped_ctrl src/main.cpp  ${SRC_LIST} )
add_dependencies(biped_ctrl ${catkin_EXPORTED_TARGETS})

# ${catkin_EXPORTED_TARGETS}: 
# 包含了当前ROS软件包所依赖的其他软件包导出的目标或依赖项
# 的名称。${catkin_EXPORTED_TARGETS}变量是由catkin_package()命令自动生成的，其中包含了所有依赖软件包导出的目标或依赖项的名称。

target_link_libraries(biped_ctrl ${catkin_LIBRARIES} qpOASES quadprog pinocchio::pinocchio  -pthread lcm tsid::tsid)
# target_link_libraries(biped_ctrl  )


