# CMake最低版本要求
cmake_minimum_required(VERSION 3.0.2)
project(pcd_saver)

# 找到catkin的包
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  sensor_msgs
  roscpp
)

# 找到PCL的包
find_package(PCL 1.8 REQUIRED)

# 包的配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pcd_saver
  CATKIN_DEPENDS pcl_ros sensor_msgs roscpp
)

# 设置编译的头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 编译源文件并生成可执行文件
add_executable(pcd_saver_node src/pcd_saver.cpp)

# 链接catkin和PCL的库
target_link_libraries(pcd_saver_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 安装时指定可执行文件的位置
install(TARGETS pcd_saver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
