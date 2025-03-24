#!/bin/bash

container_name="RS_sense"
docker stop $container_name
work_space=$1
# 检查容器是否已经存在
if [ "$(docker ps -aq -f name=$container_name)" ]; then
    echo "Container already exists. Starting the container..."
    docker start $container_name
else
    echo "Creating and starting a new container..."
    docker run -it -d --rm --name $container_name --net=host -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all --gpus all -e DISPLAY=$DISPLAY -v /tmp:/tmp -v /dev/aks/:/dev/aks/ --privileged -v $work_space:/home/sti/RS_sense_v3 rs_sense_cuda11.8:v1.0.0 
fi

# AI节点
gnome-terminal -- bash -c "
    docker exec -it $container_name /bin/bash -c '
        source /opt/ros/noetic/setup.bash &&
        cd ./RS_sense_v3 &&
        export LD_LIBRARY_PATH=./application/libs/lib/x86_64:\$LD_LIBRARY_PATH &&
        export LD_LIBRARY_PATH=./third_party/locker/x86_64:\$LD_LIBRARY_PATH &&
        ./build/application/RS_sense_frontbev;
        exec bash
    ';
    exit"

# 等待直到终端关闭

# 跟踪节点
gnome-terminal -- bash -c "
    docker exec -it $container_name /bin/bash -c '
        source /opt/ros/noetic/setup.bash &&
        cd ./RS_sense_v3 &&
        export LD_LIBRARY_PATH=./application/libs/lib/x86_64:\$LD_LIBRARY_PATH &&
        export LD_LIBRARY_PATH=./third_party/locker/x86_64:\$LD_LIBRARY_PATH &&
        ./build/application/RS_sense_tracking;
        exec bash
    ';
    exit"

# 可视化节点
gnome-terminal -- bash -c "
    docker exec -it $container_name /bin/bash -c '
        source /opt/ros/noetic/setup.bash &&
        cd ./RS_sense_v3 &&
        export LD_LIBRARY_PATH=./application/libs/lib/x86_64:\$LD_LIBRARY_PATH &&
        export LD_LIBRARY_PATH=./third_party/locker/x86_64:\$LD_LIBRARY_PATH &&
        ./build/application/ros/rviz_display_manager/rviz_display_node;
        exec bash
    ';
    exit"

# rviz 节点
gnome-terminal -- bash -c "
    docker exec -it $container_name /bin/bash -c '
        source /opt/ros/noetic/setup.bash &&
        cd ./RS_sense_v3 &&
        rviz -d ./rviz/perception.rviz;
        exec bash
    ';
    exit"

read -p "Press any key to stop the container and exit..."

# 停止并删除容器
docker stop $container_name
# pkill -f 'gnome-terminal'