import cv2
import numpy as np

import sys

import HaiKang

current_dir = sys.path[0]
sys.path.insert(0, current_dir)
sys.path.append(r".\MvImport")
from MvCameraControl_class import *

def adjust_canny_thresholds():
    def nothing(x):
        pass

    # 创建窗口
    cv2.namedWindow('Canny Edge Detection')

    # 创建两个轨迹条，分别用于调整低阈值和高阈值
    cv2.createTrackbar('Min Threshold', 'Canny Edge Detection', 0, 255, nothing)
    cv2.createTrackbar('Max Threshold', 'Canny Edge Detection', 0, 255, nothing)

    # 打开摄像头
    # 获得设备信息
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    # ch: 枚举设备 | en:Enum device
    # nTLayerType[IN] 枚举传输层 ，pstDevList[OUT] 设备列表
    HaiKang.Enum_device(tlayerType, deviceList)

    # 获取相机和图像数据缓存区
    cam, data_buf, nPayloadSize = HaiKang.enable_device(0, deviceList)  # 选择第一个设备

    while True:

        # 读取摄像头画面
        image = HaiKang.get_image(cam, data_buf, nPayloadSize)
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.imshow("image", image)

        # 转换为灰度图像
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 获取当前轨迹条位置
        min_thresh = cv2.getTrackbarPos('Min Threshold', 'Canny Edge Detection')
        max_thresh = cv2.getTrackbarPos('Max Threshold', 'Canny Edge Detection')

        # 应用Canny边缘检测
        edges = cv2.Canny(gray, min_thresh, max_thresh)

        # 显示结果
        cv2.imshow('Canny Edge Detection', edges)

        # 按下ESC键退出
        if cv2.waitKey(1) == 27:
            break

    # 释放摄像头并销毁所有窗口
    HaiKang.close_device(cam, data_buf)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    adjust_canny_thresholds()
