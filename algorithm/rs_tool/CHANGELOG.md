# rs_tool changelog

**3.1.0(2023-1-6) by YanzhiWei**

Fix: fix bug of roi/perception's range display 

**3.1.0(2022-11-16) by ChenYingfo**

Fix: fix bug of roi display

**3.1.0(2022-11-16) by Niyifan**

Fix: 
1. fix localizationsender init not return bug 
2. fix localizationsender proto sender bug

**3.1.0(2022-10-10) by ChenYingfo**

Fix: fix communication docs

**3.1.0(2022-10-10) by Brooks Wu**

Feat: adapt pose_stamped in sensor2

**3.1.0(2022-09-25) by ChenYingfo**

Fix: fix compiling error of websocket under ubuntu 2004.

**3.1.0(2022-09-21) by WeiYanzhi**

Fix: fix object's roi_id bug

**3.1.0(2022-09-19) by ChenYingfo**

Fix: 修复websocket通信方式的bug。

**3.1.0(2022-09-07) by ChenYingfo**

Fix: 修复通信模块发送点云时会出现内存越界的问题。

**3.1.0(2022-09-02) by NiYifan**

Fix:
1.V2R UDP_SENDER、TCP_CLIENT、TCP_SERVER测试，bug修复
2.增加V2R结果打印

**3.1.0(2022-08-31) by NiYifan**

Feat: 增加v2r 1.4、1.5、1.6各版本的反序列化及接收功能

**3.1.0(2022-08-31) by ChenYingfo**

Fix: 修复rs_sensor发送packets时工程崩溃的问题。

**3.1.0(2022-08-26) by ChenYingfo**

Fix: 修复P系列没有传入localization mode以及pose guess的问题。

**3.1.0(2022-08-25) by WeiYanzhi**

Feat: 通信模块新增适用于TCP client和TCP server的接收代码。

**3.1.0(2022-08-25) by NiYifan**

Feat:
1. 增加 protobuf 点云接收及反序列化功能
2. 发送端更改单次打包点云数以及长度状态计数标签修改。

**3.1.0(2022-08-22) by NiYifan**

Feat:
1. 通信模块 protobuf 通信协议新增接收及反序列化功能

**3.1.0(2022-08-18) by ChenYingfo**

Feat:
1. 通信模块 native_bytes_3.0 通信协议新增接收及反序列化功能
2. 通信模块 V2R 通信协议修复ROI的ID全部是0XFFFF的问题

**3.1.0(2022-08-11) by ChenYingfo**

Feat:
1. 通信模块 native_bytes_3.1 通信协议新增反序列化功能。
2. 新增一个调用接收端的demo。

**3.1.0(2022-08-10) by ChenYingfo**

Fix: 修复V2R全局坐标系下ROI显示不正确的问题

**3.1.0(2022-08-03) by ChenYingfo**

Fix:
1. V2R协议修复速度为0时航向角固定为90度的问题
2. V2R协议修复发送的设备ID永远为0的问题

**3.1.0(2022-08-01) by ChenYingfo**

Fix:
1. 修复通信模块用proto发送定位信息的代码编译问题。
2. 修复大唐高鸿V1.2协议下频繁的软件崩溃问题。

**3.1.0(2022-07-27) by ChenYingfo**

Fix: 修复通信模块3.1字节序方式的消息头问题

**3.1.0(2022-07-13) by ChenYingfo**

Fix: 修复通信模块proto编译不通过的问题

**3.1.0(2022-06-20) by ChenYingfo**

Fix: 修复大唐高鸿Json V1.2 协议的时间戳错误问题

**3.1.0(2022-04-01) by ChenYingfo**

Fix: 修复ROI角点过滤失效的bug。

**3.1.0(2022-03-28) by ChenYingfo**

Fix: 修复P系列及V2R中，object的轨迹/历史速度/历史类型为空的问题。

**3.1.0(2022-03-25) by ChenYingfo**

Fix: 修复object信息中存在polygon为空的问题。

**3.1.0(2022-03-08) by ChenYingfo**

Fix: 修复V2R在添加了全局pose的情况下感知范围显示不正确的问题。

**3.1.0(2022-02-25) by ChenYingfo**

Fix:

1. 修复部分复杂ROI显示不正确的问题。
2. ros通信方式中删去无用msg内容。

**3.1.0(2022-01-27) by ChenYingfo**

Fix:修复ROI有关物体过滤的问题，完善ROI过滤的功能。

**3.1.0(2022-01-27) by ChenYingfo**

Fix:修复RVIZ显示marker array的时候出现warning status和error status的问题。

**3.1.0(2022-01-06) by ChenYingfo**

Feat: rviz新增object的XYZ坐标显示

**3.1.0(2022-01-05) by ChenYingfo**

Fix: 修复ros及ros2通信方式中，object的coreinfo里面的时间戳序列化错误的问题。

**3.1.0(2021-12-22) by ChenYingfo**

Fix: 修复原始点云topic在echo的时候时间戳显示格式不正确的问题。

**3.1.0(2021-12-20) by ChenYingfo**

Feat:

1. 通信模块新增感知消息protobuf序列化方式以及ROS2序列化方式。
2. 通信模块新增ROS2发送及接收方式。
3. localization sender部分新增定位消息protobuf序列化方式并采用udp进行发送。

Fix:

1. 更正json的include方式不正确导致工程在ros2环境下无法编译通过的问题。
2. 补充部分代码的版权声明。

**3.1.0(2021-12-20) by ChenYingfo**

Feat: 通信模块新增SDK3.1感知消息小端字节序序列化方式。

**3.1.0(2021-12-10) by ChenYingfo**

Fix: 修复通信模块使用ros通信方式时，rostopic echo 持续打印已消失物体的问题。

**3.1.0(2021-12-08) by Owen**

Feat: support fusion lidar send

**3.1.0(2021-11-30) by ChenYingfo**

Fix:

1. 修复通信模块中客户定制消息处理逻辑的bug。
2. 修复rviz显示在终端报warning的问题。

Feat: P系列增加 map filter 接口。

**3.1.0(2021-11-23) by ChenYingfo**

Fix: 3.1工程中ros通信里percept_topic的问题

**3.1.0(2021-11-19) by ChenYingfo**

Fix: 修复大唐高鸿自定义消息中ID不稳定的错误

**3.1.0(2021-11-17) by Owen**

Fix: 修复找不到boost编译错误

**3.1.0(2021-11-03) by Owen**

Feat: 新增mirror detection支持，更新rviz显示

**3.1.0(2021-10-28) by ChenYingfo**

Fix:

1.  更正事件检测模块中“车辆逆行”部分的错误。
2.  根据技术支持的反馈更正事件检测模块中有关“横穿”和“闯入”事件的错误。
3.  事件检测模块更新“连续变道”事件的判断逻辑。
4.  修正大唐高鸿自定义消息发送频率不均的问题。

**3.1.0(2021-10-21) by ChenYingfo**

Fix: 更正大唐高鸿两种序列化方式的命名空间重复导致的代码无法正常运行的错误。

**3.1.0(2021-10-15) by ChenYingfo**

Feat: 通信模块新增大唐高鸿序列化成Json的方式并适配调整后的发送方式。

**3.1.0(2021-10-14) by ChenYingfo**

Feat: 通信模块增加大唐高鸿数据记录软件序列化方式

Perf:  

1. 调整并精简通信模块发送/接收方式
2. 通信模块按照SDK3.1 release的风格整合V2R和Websocket通信方式。
3. rs_beta中针对大唐高鸿数据记录软件需求调整map_filter的输出。

**3.1.0(2021-10-13) by Owen**

Fix: 修复map filter 编译问题

**3.1.0(2021-10-12) by Owen**

Fix: 修复rviz 没有ros环境编译问题

**3.1.0(2021-10-11) by Owen**

Feat: copy ground filter result

**3.1.0(2021-09-28) by Owen**

Feat: multi thread for roaddetection basicdetection and scenestruct

**3.1.0(2021-09-27) by Owen**

Feat: multi thread for roaddetection and basicdetection

**3.1.0(2021-09-26) by Owen**

Feat: add external pose sensor

**3.1.0(2021-09-24) by Owen**

Feat: 适配localization_interface接口目录调整


**3.1.0(2021-09-23) by Chenyingfo**

Fix : rviz 修复 pcd 地图显示问题。

Feat :  rviz 增加显示坐标系的配置接口。

**3.1.0(2021-09-22) by Owen**

Fix: 修复理想one can信息接收错误

**3.1.0(2021-09-18) by Owen**

Feat: 添加pcap依赖库

**3.1.0(2021-09-16) by Owen**

Fix: 修改p系列初始化顺序问题

**3.1.0(2021-09-16) by Owen**

Perf: 调整pcap 配置参数使用逻辑

**3.1.0(2021-09-15) by Owen**

Feat: add pcap support 

**3.1.0(2021-09-13) by Owen**

Feat: imu and odom add new source from gnss

**3.1.0(2021-09-13) by Owen**

Feat: v2r 移除road detection 和 basic detection

**3.1.0(2021-09-13) by Owen**

Perf: rviz显示改为异步方式，可通过配置文件选择是否开启

**3.1.0(2021-09-13) by Owen**

Fix: 修复跟踪预测结果赋值问题

**3.1.0(2021-09-10) by Owen**

Feat: update application

**3.1.0(2021-09-09) by Owen**

Feat: update pose related

**3.1.0(2021-09-06) by Owen**

Fix: 修复dependence cmake bug, 修复点云为空时显示崩溃

**3.1.0(2021-09-03) by Owen**

Feat: 修复box显示逻辑

**3.1.0(2021-09-03) by Owen**

Feat: rm send_fusion_cloud

**3.1.0(2021-09-01) by Owen**

Feat: update result sender

**3.1.0(2021-08-31) by Wujixiu**

Fix: add functional header files

**3.1.0(2021-08-27) by Moujiajun**

Feat: update the updates in old version

**3.1.0(2021-08-25) by Owen**

Fix: fix no return error in sender

**3.1.0(2021-08-25) by Owen**

Feat: remove basic detection and road detection in v2r

**3.1.0(2021-08-25) by Owen**

Feat: update pseries application

**3.1.0(2021-08-24) by Moujiajun**
**3.1.0(2021-09-10) by Owen**

Feat: update application

**3.1.0(2021-09-09) by Owen**

Feat: update pose related

**3.1.0(2021-09-06) by Owen**

Fix: 修复dependence cmake bug, 修复点云为空时显示崩溃

**3.1.0(2021-09-03) by Owen**

Feat: 修复box显示逻辑

**3.1.0(2021-09-03) by Owen**

Feat: rm send_fusion_cloud

**3.1.0(2021-09-01) by Owen**

Feat: update result sender

**3.1.0(2021-08-31) by Wujixiu**

Fix: add functional header files

**3.1.0(2021-08-27) by Moujiajun**

Feat: update the updates in old version

**3.1.0(2021-08-25) by Owen**

Fix: fix no return error in sender

**3.1.0(2021-08-25) by Owen**

Feat: remove basic detection and road detection in v2r

**3.1.0(2021-08-25) by Owen**

Feat: update pseries application

**3.1.0(2021-08-24) by Moujiajun**

Fix: 1. fix a bug when receive pointcloud

**3.1.0(2021-08-23) by Moujiajun**

Feat: 1. remove all the submodules and keep only one project for rs_tool
