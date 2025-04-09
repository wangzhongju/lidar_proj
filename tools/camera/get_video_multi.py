import cv2
import time
import os
from multiprocessing import Process
import sys

def get_cam_name(rtsp_url):
    """从RTSP地址提取摄像头标识"""
    return rtsp_url.split("@")[-1].replace(":", "_").replace("/", "_")

def rtsp_worker(rtsp_url, base_dir="output", interval=1,duration=5):
    """
    单路RTSP视频处理进程
    :param rtsp_url: RTSP流地址
    :param base_dir: 输出根目录
    :param interval: 抽帧间隔(秒)
    """
    cam_name = get_cam_name(rtsp_url)
    output_dir = os.path.join(base_dir, cam_name)
    os.makedirs(output_dir, exist_ok=True)
    
    last_capture = time.time()
    cap = None
    reconnect_attempts = 0
    record_cnt=0
    max_cnt=int(duration)
    while True:
        try:
            if cap is None or not cap.isOpened():
                print(f"[{cam_name}] 正在连接...")
                cap = cv2.VideoCapture(rtsp_url)
                if not cap.isOpened():
                    raise ConnectionError("连接失败")

            ret, frame = cap.read()
            if not ret:
                raise RuntimeError("帧数据异常")
            
            current_time = time.time()
            if current_time - last_capture >= interval:
                # 生成唯一文件名
                timestamp = time.time()
                filename = f"{cam_name}_{timestamp}.png"
                save_path = os.path.join(output_dir, filename)
                
                # 保存PNG（压缩级别6，平衡速度与体积）
                cv2.imwrite(save_path, frame, [cv2.IMWRITE_PNG_COMPRESSION, 6])
                print(f"[{cam_name}] 已保存：{filename}")
                
                last_capture = current_time
                reconnect_attempts = 0  # 重置重连计数器
                record_cnt=record_cnt+1
            if record_cnt >= max_cnt:
                break

        except Exception as e:
            print(f"[{cam_name}] 错误：{str(e)}")
            reconnect_attempts += 1
            
            # 重连逻辑
            if cap is not None:
                cap.release()
                cap = None
                
            if reconnect_attempts > 3:
                print(f"[{cam_name}] 超过最大重试次数，终止进程")
                break
                
            time.sleep(min(2**reconnect_attempts, 10))  # 指数退避

if __name__ == "__main__":
    # 配置摄像头列表（示例）
    rtsp_list = [
        "rtsp://admin:nebula2025@10.28.49.69",
        "rtsp://admin:nebula2025@10.28.49.70",
        "rtsp://admin:nebula2025@10.28.49.71"
    ]

    processes = []
    for url in rtsp_list:
        p = Process(target=rtsp_worker, args=(url, sys.argv[1], 1, sys.argv[2]))
        p.start()
        processes.append(p)

    try:
        for p in processes:
            p.join()
    except KeyboardInterrupt:
        print("\n检测到中断信号，终止所有进程...")
        for p in processes:
            p.terminate()

