#!/bin/bash

# 检查是否传递了读取路径和保存路径
if [ -z $1 ] || [ -z $2 ]; then
    echo "请提供两个路径：读取路径和保存路径"
    exit 1
fi

# 获取读取路径和保存路径
read_path=$1
save_path=$2

# 检查读取路径是否有效且是一个目录
if [ ! -d $read_path ]; then
    echo "读取路径不是有效的文件夹：$read_path"
    exit 1
fi

# 检查保存路径是否有效且是一个目录
if [ ! -d $save_path ]; then
    echo "保存路径不是有效的文件夹：$save_path"
    exit 1
fi

# 遍历读取路径中的直接子文件夹
for folder in $read_path/*/; do
    if [ -d $folder ]; then
        # 获取子文件夹的名称
        folder_name=$(basename "$folder")
        
        # 在保存路径中创建同名的子文件夹
        mkdir -p $save_path/$folder_name
        echo "创建了文件夹：$save_path/$folder_name"
        cd /media/sti/work/Ives.zhang/workspace/m1p/data_preprocess/build
        ./parser $folder $save_path/$folder_name
    fi
done
