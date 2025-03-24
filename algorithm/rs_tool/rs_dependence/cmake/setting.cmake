#========================
# setting ros dependence
#========================
if (RS_ROS_DEPENDENCE)
    set(ros_dependence ON)
else()
    set(ros_dependence OFF)
endif ()


#========================
# setting can dependence
#========================
set(can_dependence ON)

#========================
# setting web socket dependence
#========================
#set(ENABLE_WEBSOCKET_COMM ON)   # "Enable Robosense Websocket Comm"
if (ENABLE_WEBSOCKET_COMM)
    set(websocket_comm ON)
else()
    set(websocket_comm OFF)
endif ()

#========================
# setting protobuf dependence
#========================
if (ENABLE_PROTOBUF_COMM)
    set(protobuf_comm ON)
else()
    set(protobuf_comm OFF)
endif ()

#========================
# setting ros2 dependence
#========================
if (ENABLE_ROS2_COMM)
    set(ros2_comm ON)
else()
    set(ros2_comm OFF)
endif ()