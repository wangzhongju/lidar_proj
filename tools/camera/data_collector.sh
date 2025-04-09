#!/bin/bash

collector_name=$1
duration=$2

if [ $# -eq 2 ];then
    echo "-----------------------"
else
    echo "输入参数有误，用法举例: sh $0 tuoguabeixiang_20250401-1 10"
    exit 0
fi

if test -d "/home/mic-733ao/data_collector/$1"; then
    echo "目录:/home/mic-733ao/data_collector/$1,存在,请重新输入名称"
    exit 0
else
    echo "start"
fi

v_duration=$(($duration + 1))
output_dir=/home/mic-733ao/data_collector/data
mkdir -p $output_dir
cd $output_dir
echo "开始录制rosbag包和视频抽帧,时间$duration秒......保存目录为:$1"
python3 /home/mic-733ao/shell/get_video.py $output_dir $v_duration &
rosbag record --duration=$2 /75/rslidar_points /76/rslidar_points /77/rslidar_points &


sleep_int=$(($duration + 10))
echo $sleep_int
sleep $sleep_int

echo "开始对rosbag结果数据进行抽帧................"
pcd_dir=$output_dir/pcd
mkdir -p $pcd_dir
/home/mic-733ao/rosbag2pcd/data_preprocess/build/parser $output_dir $pcd_dir

cd /home/mic-733ao/data_collector/data/
rm -f *.bag


cd /home/mic-733ao/data_collector/
mv data $collector_name
tar -zcvf $collector_name.tar.gz $collector_name
mv $collector_name.tar.gz output/
echo "采集完毕按任意键结束......." 
