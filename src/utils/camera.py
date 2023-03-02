
# -*- coding: utf-8 -*-

import cv2
import os
from datetime import datetime

def create_directory(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

def save_frame(frame, photo_count, save_directory):
    filename = os.path.join(save_directory, "photo_{}.jpg".format(photo_count))
    cv2.imwrite(filename, frame)
    print("Saved {}".format(filename))

def main():
    # 指定RTSP URL
    rtsp_url = "rtsp://127.0.0.1:8554/live"
    
    # 获取用户主目录路径
    home_directory = os.path.expanduser("~")
    save_directory = os.path.join(home_directory, "photos")

    # 创建保存照片的目录
    create_directory(save_directory)

    cap = cv2.VideoCapture(rtsp_url)

    # 从本地视频文件读取
   # cap = cv2.VideoCapture("/home/xty/sentry_blue.avi")

    if not cap.isOpened():
        print("Error opening video stream or file")
        return

    photo_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # cv2.imshow('Frame', frame)

        # 检测键盘输入
        user_input = raw_input("Press 's' to save the frame, 'q' to quit: ").strip().lower()
        if user_input == 's':  # 输入 's' 保存照片
            save_frame(frame, photo_count, save_directory)
            photo_count += 1
        elif user_input == 'q':  # 输入 'q' 退出
            breakk

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
