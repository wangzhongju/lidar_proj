# ros_rtsp
订阅ROS图像主题，或者其他视频源，转换为RTSP视频源提供。




## 依赖
- ROS

- gstreamer libs:
```bash
sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev libgstrtspserver-1.0-dev gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad
```



## 编译

```bash
# 将ros_to_rtsp文件夹放置到任意位置
# 终端切换到ros_to_rtsp文件夹内
catkin_make pkg:=ros_rtsp
```



## 修改配置

配置文件如下：

```yaml
streams:

  stream1:
    type: topic  # topic - Image is sourced from a sensor_msgs::Image topic
    source: /camera_30/image_raw  # 订阅的 ROS topic
    mountpoint: /back1      # 设置rtsp流的挂载点. rtsp://<server_ip>/back1
    caps: video/x-raw,framerate=10/1,width=1920,height=1080  # 设置在获取 ROS 图像之后和 x265 编码器之前相关参数。
    bitrate: 500
    
  stream2:
    type: topic  # topic - Image is sourced from a sensor_msgs::Image topic
    source: /camera_31/image_raw  # 订阅的 ROS topic
    mountpoint: /back2      # 设置rtsp流的挂载点. rtsp://<server_ip>/back2
    caps: video/x-raw,framerate=10/1,width=1920,height=1080  # 设置在获取 ROS 图像之后和 x265 编码器之前相关参数。
    bitrate: 500
```


## 启动

```bash
cd /home/${USER}/catkin_ws
source /devel/setup.bash
roslaunch ros_rtsp rtsp_streams.launch
```



## 验证

### gstreamer
使用“gst-launch-1.0”。需要为客户端系统安装 gstreamer.  https://gstreamer.freedesktop.org/documentation/installing/index.html
```bash
gst-launch-1.0 -v rtspsrc location=rtsp://192.168.11.89:8554/back1 drop-on-latency=true use-pipeline-clock=true do-retransmission=false latency=0 protocols=GST_RTSP_LOWER_TRANS_UDP ! rtph264depay ! h264parse ! avdec_h264 ! autovideosink sync=true
```

### VLC
vlc软件内 媒体 --> 打开网络串流 ：输入rtsp://192.168.11.89:8554/back1

加载视频成功即可。

### 路侧融合感知系统

修改配置文件中CameraURI地址为：rtsp://192.168.11.89:8554/back1

启动程序，验证是否正常拉流。




## 限制
- 如果同时转换多个topic，降低分辨率以减轻延迟
- 帧率仅支持framerate=10/1，即10fps，过高会导致缓冲区错误
- 如果丢帧过多，可能网络带宽较低，尝试降低比特率bitrate
- 不支持压缩格式sensor_msgs/CompressedImage消息，支持sensor_msgs/Image格式消息





