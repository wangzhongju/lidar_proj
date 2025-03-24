#!/bin/bash


function parting_line() {
  str="="
  title=$@
  screen_length=$(stty size |awk '{print $2}')
  # screen_length=80
  title_length=$(echo -n ${title} |wc -c)
  # title_length+=1
  line_length=$(((${screen_length} - ${title_length} - 2) / 2 ))
  yes ${str} |sed ''''${line_length}'''q' |tr -d "\n" && echo -n " ${title} " && yes ${str} |sed ''''${line_length}'''q' |tr -d "\n" && echo
}

function check_build_path() {
  parting_line init env
  if [ ! -d build/ ];then
    echo "-- create build folder."
    mkdir -p build
  else
    echo "-- build path existed."
  fi
}

function build() {
  if [ -d build/ ]; then
    parting_line "build already, pass"
    return
  fi
  parting_line build
  cmake -Wno-dev -B build
  cpu_num=`cat /proc/cpuinfo| grep "processor" | sort | uniq | wc -l`
  cmake --build build -j${cpu_num}
  if [ $? != 0 ]; then
    parting_line build error
    sudo chown $USER:$USER -R build
    exit -1
  fi
  
  sudo chown $USER:$USER -R build
  parting_line build end
  cd - 1>/dev/null 2>&1
}

function source_env() {
    source /workspace/build/devel/setup.bash
    
}

function driver_run() {
    node_name="rslidar_sdk_node" 

    # 检查 ROS node 是否已经存在
    if rosnode list | grep -q "$node_name"; then
        echo "Node $node_name 已经存在，跳过启动程序"
    else
        echo "Node $node_name 不存在，启动程序"
        roslaunch rslidar_sdk start.launch &
    fi
}

function driver_run() {
    node_name="rslidar_sdk_node" 

    # 检查 ROS node 是否已经存在
    if rosnode list | grep -q "$node_name"; then
        echo "Node $node_name 已经存在，跳过启动程序"
    else
        echo "Node $node_name 不存在，启动程序"
        roslaunch rslidar_sdk start.launch &
    fi
}

function app_run() {
    node_name="rs_sdk_demo" 

    # 检查 ROS node 是否已经存在
    if rosnode list | grep -q "$node_name"; then
        echo "Node $node_name 已经存在，跳过启动程序"
    else
        echo "Node $node_name 不存在，启动程序"
        ./build/demo/rs_sdk_demo
    fi
}


function main() {

    # parting_line "build lidar driver"
    # source /opt/ros/noetic/setup.bash
    # # check_build_path
    # build

    # parting_line "build lidar process"
    # cd /workspace/algorithm
    # # check_build_path
    # build

    ros_version=$(ls -d /opt/ros/*/ 2>/dev/null | head -n 1 | xargs basename)
    if [ -z "$ros_version" ]; then
        echo "获取 ROS 版本失败，退出脚本。"
        exit 1
    fi
    source /opt/ros/$ros_version/setup.bash
    parting_line "build lidar tools process"
    # check_build_path
    catkin_make
    source devel/setup.bash

    roslaunch my_rtsp_streamer rtsp_streamer.launch &

}

main $@