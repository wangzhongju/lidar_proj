set(RS_DEP_INCLUDE_DIRS "")
set(RS_DEP_LINKER_LIBS "")
set(pcl_not_found TRUE)
#========================
# Ros part
#========================
if(ros_dependence)
    find_package(roscpp 1.12 QUIET COMPONENTS)
    find_package(pcl_ros QUIET)
    find_package(cv_bridge QUIET)

    if(roscpp_FOUND AND pcl_ros_FOUND AND cv_bridge_FOUND)
        set(pcl_not_found FALSE)
        message(=============================================================)
        message("-- RS_DEPENDENCE:ROS Found, ROS Support is turned ON!")
        message(=============================================================)
        set(ROS_FOUND "#define RS_ROS_FOUND")
        list(APPEND RS_DEP_INCLUDE_DIRS ${roscpp_INCLUDE_DIRS})
        list(APPEND RS_DEP_INCLUDE_DIRS ${pcl_ros_INCLUDE_DIRS})
        list(APPEND RS_DEP_INCLUDE_DIRS ${cv_bridge_INCLUDE_DIRS})
        list(APPEND RS_DEP_LINKER_LIBS ${roscpp_LIBRARIES})
        list(APPEND RS_DEP_LINKER_LIBS ${pcl_ros_LIBRARIES})
        list(APPEND RS_DEP_LINKER_LIBS ${cv_bridge_LIBRARIES})
    else()
        message(=============================================================)
        message("-- RS_APPLICATION:ROS Not Found, ROS Support is turned OFF!")
        message(=============================================================)
    endif()
else()
    message(=============================================================)
    message("-- RS_APPLICATION: ROS Support Not Enable!")
    message("-- RS_COMMUNICATION: ROS Communicator Method Not Enable!")
    message(=============================================================)
endif()

#========================
# Can part
#========================
if (can_dependence)
    if(EXISTS "/usr/include/canlib.h")
        set(KVASER_CAN_FOUND "#define RS_KVASER_CAN_FOUND")
        list(APPEND RS_DEP_LINKER_LIBS libcanlib.so)
    endif()
endif ()
if(EXISTS "/usr/include/pcap.h")
    list(APPEND RS_DEP_LINKER_LIBS pcap)
endif()

#========================
# WebSocket part
#========================
if (websocket_comm)
    find_package(Boost QUIET)
    if (pcl_not_found)
        find_package(pcl_ros)
    endif ()

    if (Boost_FOUND AND pcl_ros_FOUND)
        set(boost_and_pcl_found TRUE)
        set(boost_and_pcl_found ${boost_and_pcl_found} PARENT_SCOPE)

        set(WEBSOCKET_FOUND "#define ROBOSENSE_WEBSOCKET_FOUND")
        list(APPEND RS_DEP_INCLUDE_DIRS ${Boost_INCLUDE_DIR})
        list(APPEND RS_DEP_LINKER_LIBS ${Boost_LIBRARY_DIR})

        if (pcl_not_found)
            list(APPEND RS_DEP_INCLUDE_DIRS ${pcl_ros_INCLUDE_DIRS})
            list(APPEND RS_DEP_LINKER_LIBS ${pcl_ros_LIBRARIES})
        endif ()
        add_definitions(${pcl_ros_DEFINITIONS})
        set(COMPILE_WEBSOCKET TRUE)
        message(=============================================================)
        message("-- RS_COMMUNICATION: Boost And PCL Found, Support Websocket Communicater Method ")
        message(=============================================================)
    else()
        message(=============================================================)
        message("-- RS_COMMUNICATION: Boost Or/And PCL Not Found, Not Support Websocket Communicater Method ")
        message(=============================================================)
    endif ()
else()
    message(=============================================================)
    message("-- RS_COMMUNICATION: Websocket Communicator Method Not Enable!")
    message(=============================================================)
endif ()

#========================
# Protobuf part
#========================
if (protobuf_comm)
    find_package(Protobuf REQUIRED)

    if (Protobuf_FOUND)
        message(=============================================================)
        message("-- RS_COMMUNICATION:Protobuf Found, Protobuf Support is turned ON!")
        message(=============================================================)
        set(proto_found TRUE)
        set(proto_found ${proto_found} PARENT_SCOPE)

        set(PROTO_FOUND "#define RS_PROTO_FOUND")
        list(APPEND RS_DEP_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS})
        list(APPEND RS_DEP_LINKER_LIBS ${PROTOBUF_LIBRARY})
    else()
        message(=============================================================)
        message("-- RS_COMMUNICATION:Protobuf Not Found, Protobuf Support is turned off!")
        message(=============================================================)
    endif ()
else()
    message(=============================================================)
    message("-- RS_COMMUNICATION: Protobuf Support is NOT ENABLE!")
    message(=============================================================)
endif ()

#========================
# ROS2 part
#========================
if (ros2_comm)
    find_package(rclcpp QUIET)
    if (rclcpp_FOUND)
        message(=============================================================)
        message("-- RS_COMMUNICATION: Ros2 Found, Support Ros2 Communicater Method!")
        message(=============================================================)

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

        set(ros2_found TRUE)
        set(ros2_found ${ros2_found} PARENT_SCOPE)
        set(ROS2_FOUND "#define RS_ROS2_FOUND")

        list(APPEND RS_DEP_INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS})
        list(APPEND RS_DEP_LINKER_LIBS ${rclcpp_LIBRARIES})

        set(PERCEPTION_ROS2_MSG "")
        list(APPEND PERCEPTION_ROS2_MSG ${CMAKE_SOURCE_DIR}/rs_tool/rs_custom/message/robosense/ros2/perception_ros2_msg/lib)

        foreach (dir ${PERCEPTION_ROS2_MSG})
            file(GLOB_RECURSE tmp_srcs ${dir}/*.so)
            list(APPEND RS_DEP_LINKER_LIBS ${tmp_srcs})
        endforeach ()

    else()
        message(=============================================================)
        message("-- RS_COMMUNICATION:ROS2 Not Found, ROS2 Support is turned off!")
        message(=============================================================)
    endif ()
else()
    message(=============================================================)
    message("-- RS_COMMUNICATION:ROS2 Support Not Enable!")
    message(=============================================================)
endif ()