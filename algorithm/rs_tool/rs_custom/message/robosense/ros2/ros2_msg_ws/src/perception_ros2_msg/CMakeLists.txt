cmake_minimum_required(VERSION 3.5)
project(perception_ros2_msg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
#    "msg/Curb.msg"
#    "msg/Curbs.msg"
	"msg/AxisStatusPose.msg"    
	"msg/RoadEdge.msg"
    "msg/RoadEdges.msg"
    "msg/Curve.msg"
    "msg/EndPoints.msg"
    "msg/FreeSpaceInfos.msg"
    "msg/Indices.msg"
    "msg/Lane.msg"
    "msg/Lanes.msg"
    "msg/LidarFrameMsg.msg"
    "msg/Matrix3f.msg"
    "msg/Object.msg"
    "msg/Objects.msg"
    "msg/RsPerceptionMsg.msg"
    "msg/Point2f.msg"
    "msg/Point3d.msg"
    "msg/Point3f.msg"
    "msg/Point4f.msg"
    "msg/Pose.msg"
	"msg/PoseMap.msg"
	"msg/CoreInfo.msg"
    "msg/SupplementInfo.msg"
    DEPENDENCIES builtin_interfaces std_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
