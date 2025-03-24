#!/bin/bash

container_name="RS_sense"
work_space=$1

# 检查容器是否已经存在
if [ "$(docker ps -aq -f name=$container_name)" ]; then
    echo "Container already exists. Starting the container..."
    docker start -ai $container_name  # 使用 -ai 参数附加到容器中
else
    echo "Creating and starting a new container..."
    docker run -it --rm --name $container_name \
        --net=host \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -v /tmp:/tmp \
        -v /dev/aks/:/dev/aks/ \
        --privileged \
        -v "$work_space:/home/sti/RS_sense_v3" \
        rs_sense_cuda12.0:v1.0.0 \
        bash -c "
            cd /home/sti/RS_sense_v3 &&
            source /opt/ros/noetic/setup.bash &&
            rm -rf build &&
            mkdir build &&
            cd build &&
            cmake -DBUILD_MODULES=OFF .. &&
            make -j8 &&
            echo 'Build completed. You can exit the container now.'
        "
fi
