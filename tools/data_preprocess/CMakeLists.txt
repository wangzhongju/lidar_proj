cmake_minimum_required(VERSION 3.0.2)
project(your_project_name)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
# 查找 ROS 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_filters
  sensor_msgs  # 确保包含 sensor_msgs
  rosbag  # 添加 rosbag 组件
  # 添加其他需要的 ROS 组件
)

# 查找 PCL
find_package(PCL REQUIRED)

# 查找 Eigen3
find_package(Eigen3 REQUIRED)

# 查找 YAML
find_package(YAML-CPP REQUIRED)

set(CUR_SRCS "")
LIST(APPEND CUR_INCLUDES "include")

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)

foreach (dir ${CUR_SUB_DIR})
    file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h)
    list(APPEND CUR_SRCS ${tmp_srcs})
endforeach ()

# 包含头文件
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

# 添加可执行文件
add_executable(parser node/main.cpp ${CUR_SRCS})

# 链接库
target_link_libraries(parser
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${ROSBAG_LIBRARIES}
  stdc++fs
)

