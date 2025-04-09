import socket
import sys
import select
import perceptron_pb2
import math

NUMBER_Degree2Second = 3600
NUMBER_EV_Second_Meter = 30.83

#两帧之间的距离阈值
DIS_THRESHOLD = 3
#两帧之间的加速度阈值
ACCEL_THRESHOLD = 1
#两帧之间的航向角阈值
HEADING_THRESHOLD = 40



class NewTarget(object):
    def __init__(self):
        self.timestamp = 0
        self.deviceid = ""
        self.id = 0
        self.type = 0
        self.speed = 0.0
        self.heading = 0.0
        self.lontitude = 0.0
        self.latitude = 0.0
        self.accel = 0.0

# 解析protobuf
def parse_message(recv_data):
    head=recv_data[:4]
    #print (head)
    FrameType=recv_data[4]
    PerceptronType=recv_data[5]
    DataLength=recv_data[6:8]
    #print(DataLength)
    DataLength=int.from_bytes(DataLength,byteorder='big',signed=False)
    print(DataLength)
    Sensor_data=recv_data[8:8+DataLength]
    perceptronList = perceptron_pb2.PerceptronSet()
    perceptronList.ParseFromString(Sensor_data)
    print(len(perceptronList.perceptron))

    return head, FrameType, PerceptronType, DataLength, Sensor_data, perceptronList

# 计算距离原点的距离
def CalcuDistance(latitude, lontitude, originLatitude, originLontitude):
    del_lat = latitude - originLatitude
    del_NS = del_lat * NUMBER_Degree2Second * NUMBER_EV_Second_Meter
    
    del_lon = lontitude - originLontitude
    del_WE = del_lon * NUMBER_Degree2Second * NUMBER_EV_Second_Meter*math.cos(originLatitude)
    
    radius  = math.sqrt(math.pow(del_NS, 2) + math.pow(del_WE, 2))
    return radius

# 创建UDP套接字
# udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# udp_socket.bind(('192.168.10.211', 10090))

# # 设置非阻塞模式
# udp_socket.setblocking(0)

if __name__ == '__main__':   
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('192.168.11.89', 10011))
    # 设置非阻塞模式
    udp_socket.setblocking(0)

    # 创建键盘输入文件描述符
    keyboard_fd = sys.stdin.fileno()
    print('recv....')

    Ptcs = []
    is_id = 0
    last_time = 0
    count = 0
    fps = 0
    dis_error_count=0
    hz_error_count=0
    heading_error_count=0
    type_error_count=0
    acc_error_count = 0
    obj_heading_error_count=0
    obj_dis_error_count=0
    obj_acc_error_count = 0

    time_stamp_1 = 0

    # 指定要保存的CSV文件路径
    csv_file_data = 'data.csv'
    csv_file_res = 'res.csv'

while True:
    # 监听键盘输入
    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
    if ready:
        # 读取键盘输入
        key = sys.stdin.readline().strip()
        if key == 'q':
            # 当按下q键时退出循环
                udp_socket.close()
                # with open(csv_file_data, 'w', newline='') as file:
                #     writer = csv.writer(file)
                #     writer.writerows(Ptcs)
                senor_dis = 0
                yes_track = 0
                senor_dis_cout = 0
                car_num = 0
                with open(csv_file_res, 'w' ) as file:
                    file.write("当前目标列表总数,当前目标列表序号,时间戳,设备id,目标id,目标类型,目标速度,目标航向角,目标经度,目标纬度,目标加速度,和上一帧航向角差值,和上一帧加速度差值,和上一帧距离差值,和截至当前帧和第一帧距离最大值\n")
                    for index in range(len(Ptcs)):
                        target_distance = 0
                        type_is_error = False
                        heading_is_error = False
                        distance_is_error = False
                        accel_is_error = False
                        target_is_car = False
                        for num in range(len(Ptcs[index])):
                            fist_target = Ptcs[index][num]
                            if num ==0:
                                #仅判断第一帧目标类型 是否为机动车
                                if Ptcs[index][num].type>99 and Ptcs[index][num].type<200:
                                    target_is_car = True
                                    car_num +=1
                                msgstr = f"{len(Ptcs[index])},{num},{(fist_target.timestamp)},{(fist_target.deviceid)},{(fist_target.id)},{(fist_target.type)},{(fist_target.speed)},{(fist_target.heading)},{(fist_target.lontitude)},{(fist_target.latitude)},{(fist_target.accel)},"
                                resstr = f"{0},{0},{0},{0}\n"
                                file.write(msgstr+resstr)
                                continue
                            second_target = Ptcs[index][num-1]
                            #判断写入目标是否有异常
                            # if fist_target.id != second_target.id:
                            #     print(index,num,len(Ptcs[index]),fist_target.id,second_target.id,fist_target.deviceid,fist_target.deviceid)
                            #默认计算第一帧和该目标的所有距离   使用第一帧和后面所有帧计算，循环比较取最大值
                            fast_dis = CalcuDistance(fist_target.latitude,fist_target.lontitude,Ptcs[index][0].latitude,Ptcs[index][0].lontitude)
                            if fast_dis>target_distance:    
                                target_distance = fast_dis
                            #计算同一id相邻两帧的距离   阈值暂定为DIS_THRESHOLD米
                            dis = CalcuDistance(fist_target.latitude, fist_target.lontitude, second_target.latitude, second_target.lontitude)
                            if dis > DIS_THRESHOLD:
                                dis_error_count +=1
                                distance_is_error = True
                                print("distance is bigger than ",format(DIS_THRESHOLD)," res = ",dis," distance = ",format(dis),"target id = ",format(fist_target.id))
                            #计算同一id相邻航向角的差值 阈值暂定为40
                            heading = abs(fist_target.heading - second_target.heading)
                            if heading > 180 and heading <360:
                                heading = 360-heading
                            if heading > HEADING_THRESHOLD:
                                heading_error_count +=1
                                heading_is_error = True
                                print("heading is bigger than ",format(HEADING_THRESHOLD)," res = ",heading,"first heading = ",fist_target.heading," second heading = ",second_target.heading)
                            #计算同一id相两帧的目标类型
                            if fist_target.type != second_target.type:
                                type_is_error = True
                                print("first type = ",fist_target.type," second type = ",second_target.type," target_id = ",fist_target.id,second_target.id)
                            #计算同一id相邻两帧加速度的差值 阈值暂定为1
                            accel_abs = abs(fist_target.accel - second_target.accel)
                            if accel_abs> ACCEL_THRESHOLD:
                                acc_error_count +=1
                                accel_is_error = True
                                #print("accel_abs is bigger than ",format(ACCEL_THRESHOLD)," res = ",accel_abs,"first accel = ",fist_target.accel," second accel = ",second_target.accel," target_id = ",fist_target.id,second_target.id)
                            #保存结果数据   数据格式为：
                            #当前目标列表总数，当前目标列表序号，时间戳，设备id，目标id，目标类型，目标速度，目标航向角，目标经度，目标纬度，目标加速度，和上一帧航向角差值，和上一帧加速度差值，和上一帧距离差值，和截至当前帧和第一帧距离最大值
                            # with open(csv_file_res, 'a' ) as file:
                            msgstr = f"{len(Ptcs[index])},{num},{(fist_target.timestamp)},{(fist_target.deviceid)},{(fist_target.id)},{(fist_target.type)},{(fist_target.speed)},{(fist_target.heading)},{(fist_target.lontitude)},{(fist_target.latitude)},{(fist_target.accel)},"
                            resstr = f"{heading},{accel_abs},{dis},{target_distance}\n"
                            file.write(msgstr+resstr)
                        if type_is_error:
                            type_error_count +=1
                        if accel_is_error:
                            obj_acc_error_count +=1
                        if heading_is_error:
                            obj_heading_error_count +=1
                        if distance_is_error:
                            obj_dis_error_count +=1
                        if target_distance >100 and target_is_car:
                            yes_track +=1
                        if target_distance>100 and target_distance<300:         #计算跟踪距离时过滤距离过长和过短的目标
                            senor_dis += target_distance
                            senor_dis_cout +=1
                        elif target_distance>=300:
                            print(" target_distance is too large ,give up, target_distance = ",target_distance)
                        # else :
                        #     print(" target_distance is too small ,give up, target_distance = ",target_distance)

                if car_num !=0 :
                    print("机动车跟踪距离大于100米率 = {}".format(yes_track/car_num)) #有效跟踪目标/目标数
                else :
                    print("计算机动车跟踪距离大于100米率错误,初始目标目标类型为车的数量为0 " )
                if senor_dis != 0 and senor_dis_cout != 0:
                    print("有效感知范围（米） = {}".format(senor_dis/senor_dis_cout))   #有效距离之和/有效距离的目标数
                else:
                    print("计算有效感知范围错误,有效距离之和 = ",senor_dis," 有效距离的目标数 = ",senor_dis_cout)
                if  count!=0:
                    print("总目标帧数 = {}".format(count))
                    print("两帧距离合格率 = {}".format(1-dis_error_count/count))          #异常距离帧数/总目标数
                    print("航向角合格率 = {}".format(1-heading_error_count/count))        #异常航向角帧数/总目标数帧数
                    print("加速度合格率 = {}".format(1-acc_error_count/count))            #异常加速度帧数/总目标数
                if len(Ptcs)!=0:
                    print("总目标数 = {}".format(len(Ptcs)))
                    print("感知目标类型合格率 = {}".format(1-type_error_count/len(Ptcs)))           #异常目标数/总目标数
                    print("航向角合格目标占比 = {}".format(1-obj_heading_error_count/len(Ptcs)))    #异常目标数/总目标数
                    print("加速度合格目标占比 = {}".format(1-obj_acc_error_count/len(Ptcs)))        #异常目标数/总目标数
                    print("距离合格目标占比 = {}".format(1-obj_dis_error_count/len(Ptcs)))          #异常目标数/总目标数
                if fps !=0:
                    print("输出频率合格率 = {}".format(1-hz_error_count/fps))           #异常频率帧数/列表总行数
                sys.exit()

    try:
        # 接收UDP数据
        # data, addr = udp_socket.recvfrom(1024)
        recv_data, addr = udp_socket.recvfrom(1024*1024*2)
        head, FrameType, PerceptronType, DataLength, Sensor_data, perceptronList = parse_message(recv_data)
        Targets =[]
        if len(perceptronList.perceptron)>0:
            fps +=1
            for items in perceptronList.perceptron:
                count +=1
                target = NewTarget()
                target.deviceid = perceptronList.devide_id
                target.timestamp = perceptronList.time_stamp #时间戳
                target.id = items.object_id      #id
                target.type = items.object_class_type*100+items.ptc_veh_type        #类型  细分到车辆类型 使用目标大类*100+目标车辆类型
                target.speed = items.object_speed    #速度
                target.heading = items.object_heading       #航向角
                target.lontitude = items.point_gps.object_longitude     #经度
                target.latitude = items.point_gps.object_latitude       #纬度
                target.accel = items.object_acceleration                #加速度
                if len(Ptcs) > 0:
                    for ids in range(len(Ptcs)):
                        #要求目标id和设备id同时相同时才放进同一列表
                        if Ptcs[ids][0].id == target.id and Ptcs[ids][0].deviceid == target.deviceid:
                            # print(ids,len(Ptcs[ids]),Ptcs[ids][0].id,target.id,Ptcs[ids][0].deviceid,target.deviceid)
                            Ptcs[ids].append(target)
                            is_id = 1
                            break
                    if 1 != is_id:
                        Targets.append(target)
                        Ptcs.append(Targets)
                        Targets=[]
                else:
                    Targets.append(target)
                    Ptcs.append(Targets)
                    Targets=[]
                #保存原始数据
                # with open(csv_file_data, 'a' ) as file:
                #     msgstr = f"{(perceptronList.time_stamp)},{(perceptronList.devide_id)},{(items.object_id)},{(items.object_class_type*100+items.ptc_veh_type)},{(items.object_speed)},{(items.object_heading)},{(items.point_gps.object_longitude)},{(items.point_gps.object_latitude )},{(items.object_acceleration)}\n"
                #     file.write(msgstr)

                    # result = f"{items.object_id},{items.object_class_type*100+items.ptc_veh_type},{items.object_speed}"

                # for index in range(len(Ptcs)):
                #     for num in range(len(Ptcs[index])):
                #         if num >0:
                #             if Ptcs[index][num].id != Ptcs[index][num-1].id:
                #                 sys.exit()
                is_id = 0
            time_stamp_2 = perceptronList.time_stamp
            #两帧之间间隔大于120或者小于90都算不合格
            if time_stamp_1 !=0 and (int(time_stamp_2)-int(time_stamp_1)>120 or int(time_stamp_2)-int(time_stamp_1)<90):
                hz_error_count +=1
            
            #print('recv time gap: {time_stamp_2 - time_stamp_1}',len(perceptronList.perceptron),len(perceptronList.event_list))
            time_stamp_1 = time_stamp_2
        # print(f"Received data: {data.decode()} from {addr}")
    except socket.error:
        pass

# 关闭UDP套接字
udp_socket.close()
