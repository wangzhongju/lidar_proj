import cv2
import os
#import ffmpeg
import argparse
import datetime

class RTSPVideoCapture:
    def __init__(self, ip, usr, password, save_file=None):
        # 构建 RTSP URL
        self.ip_address = f"{ip}"
        self.rtsp_url = f"rtsp://{usr}:{password}@{ip}"
        
        if save_file is None or save_file.strip() == "":
            current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.save_path = f"./video/{ip}/{current_time}_{ip}.mp4"
        else:
            self.save_path = save_file

        if not os.path.exists(os.path.dirname(self.save_path)):
            os.makedirs(os.path.dirname(self.save_path))
        
        self.cap = cv2.VideoCapture(self.rtsp_url)
        if not self.cap.isOpened():
            raise ValueError("Error: Could not open video.")

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print("width: ", self.width)
        print("width: ", self.height)

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.output_video = cv2.VideoWriter(self.save_path, fourcc, 10.0, (self.width, self.height))
        
        # 初始化录制状态
        self.recording = False

    def start_capture(self):
        # window_name = 'RTSP Video'
        window_name = self.ip_address
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, self.width // 2, self.height // 2)

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            # 在窗口中显示帧
            # status_text = f"{window_name} {'(录制中)' if self.recording else '(录制暂停)'}"
            cv2.imshow(window_name, frame)

            # 根据录制状态写入视频
            if self.recording:
                print("recording ...")
                self.output_video.write(frame)

            # 检测按键
            key = cv2.waitKey(1)
            if key == 27:  # ESC键
                break
            elif key == ord('p'):  # 'p'键暂停/继续录制
                self.recording = not self.recording
                if self.recording:
                    print("Resuming recording...")
                else:
                    print("Recording paused.")
            elif key == ord('s'):  # 's'键保存当前帧为图片
                self.save_frame(frame)

        # 释放资源
        self.release()
    
    def save_frame(self, frame):
        # 创建以 IP 地址命名的目录
        ip_folder = f"./img/{self.ip_address.split('/')[-1]}"
        if not os.path.exists(ip_folder):
            os.makedirs(ip_folder)

        # 计算当前文件夹中已存在的文件数量，以便生成新文件名
        #existing_files = [f for f in os.listdir(ip_folder) if f.endswith('.jpg')]
        #frame_number = len(existing_files) + 1 + 2000  # 生成新文件名，基于现有文件数量
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        #image_filename = os.path.join(ip_folder, f"{frame_number}.jpg")
        image_filename = os.path.join(ip_folder, f"{timestamp}.jpg")
        cv2.imwrite(image_filename, frame)
        print(f"Saved frame as {image_filename}")

    def release(self):
        self.cap.release()
        self.output_video.release()
        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description='RTSP Video Stream Capture')
    parser.add_argument('-ip', '--ip', type=str, required=True, help='IP address (e.g., 10.28.49.43)')
    parser.add_argument('-u', '--usr', type=str, required=True, help='Username for the RTSP source (e.g., admin)')
    parser.add_argument('-p', '--password', type=str, required=True, help='Password for the RTSP source (e.g., abc12345)')
    parser.add_argument('--save_file', type=str, default="", help='Filename to save the output video (e.g., output.mp4)')
    args = parser.parse_args()

    video_capture = RTSPVideoCapture(args.ip, args.usr, args.password, args.save_file)
    video_capture.start_capture()

if __name__ == '__main__':
    main()