#========================
# libs
#========================

set(CUR_SRCS "")
set(CUR_INCLUDES "")
list(APPEND CUR_INCLUDES "third_part")

set(CUR_SUB_DIR "")
#list(APPEND CUR_SUB_DIR serialization)
LIST(APPEND CUR_SUB_DIR custom)
LIST(APPEND CUR_SUB_DIR communication)
LIST(APPEND CUR_SUB_DIR third_part/lz4)
list(APPEND CUR_SUB_DIR cloud_sender)
if (COMPILE_LOCALIZATION)
    LIST(APPEND CUR_SUB_DIR localization_sender)
endif ()

if (proto_found)
    execute_process(COMMAND protoc -I=${CMAKE_CURRENT_SOURCE_DIR}/message/robosense/proto_msg
            --cpp_out=${CMAKE_CURRENT_SOURCE_DIR}/message/robosense/proto_msg
            ${CMAKE_CURRENT_SOURCE_DIR}/message/robosense/proto_msg/Proto_msg.Percept.proto)

    execute_process(COMMAND protoc -I=${CMAKE_CURRENT_SOURCE_DIR}/message/robosense/proto_msg
            --cpp_out=${CMAKE_CURRENT_SOURCE_DIR}/message/robosense/proto_msg
            ${CMAKE_CURRENT_SOURCE_DIR}/message/robosense/proto_msg/Proto_msg.Loc.proto)
    list(APPEND CUR_SUB_DIR message/robosense/proto_msg)
endif ()

list(APPEND CUR_SUB_DIR third_part/websocket/jsoncpp)
if(boost_and_pcl_found)
    list(APPEND CUR_SUB_DIR third_part/websocket/util)
    list(APPEND CUR_SUB_DIR third_part/websocket/websocketpp)
endif()

if (ros2_found)
    list(APPEND CUR_SUB_DIR message/robosense/ros2/perception_ros2_msg/msg)
endif ()

list(APPEND CUR_INCLUDES third_part/websocket/jsoncpp)
foreach (dir ${CUR_SUB_DIR})
    file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h ${dir}/*.hpp ${dir}/*.c ${dir}/*.cc)
    list(APPEND CUR_INCLUDES ${dir}/include)
    list(APPEND CUR_SRCS ${tmp_srcs})
endforeach ()

add_library(${CUR_LIB} SHARED
        ${CUR_SRCS}
        )
target_include_directories(${CUR_LIB}
        PUBLIC
        ${CUR_INCLUDES}
        ${RS_DEP_INCLUDE_DIRS}
        )
if (RS_ROS_DEPENDENCE)
    target_include_directories(${CUR_LIB}
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/message/robosense/ros/
            )
endif ()
if (ENABLE_WEBSOCKET_COMM)
    target_include_directories(${CUR_LIB}
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/third_part/websocket/
            ${CMAKE_CURRENT_SOURCE_DIR}/third_part/websocket/util/include
            )
endif()
if (ENABLE_PROTOBUF_COMM)
    target_include_directories(${CUR_LIB}
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/message/robosense/proto_msg
            )
endif ()
if (ENABLE_ROS2_COMM)
    target_include_directories(${CUR_LIB}
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/message/robosense/ros2/
            )
endif ()
target_link_libraries(${CUR_LIB}
        PUBLIC
        rs_common
        rs_perception_common
        rs_preprocessing
        rs_dependence
        rs_beta
        rs_sensor
        ${RS_DEP_LINKER_LIBS}
        )
if (COMPILE_LOCALIZATION)
    target_link_libraries(${CUR_LIB}
            PUBLIC
            rs_localization
            )
endif()


