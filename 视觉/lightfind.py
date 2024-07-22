import cv2
import numpy as np

import sys

import HaiKang

current_dir = sys.path[0]
sys.path.insert(0, current_dir)
sys.path.append(r".\MvImport")
from MvCameraControl_class import *

def adjust_red_thresholds():
    def nothing(x):
        pass

    # 创建窗口
    cv2.namedWindow('Laser Detection')

    # 创建四个轨迹条，分别用于调整HSV的低阈值和高阈值
    cv2.createTrackbar('Lower H', 'Laser Detection', 0, 179, nothing)
    cv2.createTrackbar('Lower S', 'Laser Detection', 0, 255, nothing)
    cv2.createTrackbar('Lower V', 'Laser Detection', 0, 255, nothing)
    cv2.createTrackbar('Upper H', 'Laser Detection', 0, 179, nothing)
    cv2.createTrackbar('Upper S', 'Laser Detection', 0, 255, nothing)
    cv2.createTrackbar('Upper V', 'Laser Detection', 0, 255, nothing)

    # 初始化轨迹条位置
    cv2.setTrackbarPos('Lower H', 'Laser Detection', 35)
    cv2.setTrackbarPos('Lower S', 'Laser Detection', 100)
    cv2.setTrackbarPos('Lower V', 'Laser Detection', 100)
    cv2.setTrackbarPos('Upper H', 'Laser Detection', 85)
    cv2.setTrackbarPos('Upper S', 'Laser Detection', 255)
    cv2.setTrackbarPos('Upper V', 'Laser Detection', 255)




    try:
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
            image = HaiKang.get_image(cam,data_buf, nPayloadSize)

            x = 0
            y = 0
            # 进行高斯模糊
            blurred = cv2.GaussianBlur(image, (11, 11), 0)
            # 转换颜色空间到HSV
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # 获取当前轨迹条位置
            lower_h = cv2.getTrackbarPos('Lower H', 'Laser Detection')
            lower_s = cv2.getTrackbarPos('Lower S', 'Laser Detection')
            lower_v = cv2.getTrackbarPos('Lower V', 'Laser Detection')
            upper_h = cv2.getTrackbarPos('Upper H', 'Laser Detection')
            upper_s = cv2.getTrackbarPos('Upper S', 'Laser Detection')
            upper_v = cv2.getTrackbarPos('Upper V', 'Laser Detection')

            # 设置颜色阈值
            lower_red = np.array([lower_h, lower_s, lower_v])
            upper_red = np.array([upper_h, upper_s, upper_v])

            # 对图片进行二值化处理
            mask = cv2.inRange(hsv, lower_red, upper_red)
            # 腐蚀操作
            mask = cv2.erode(mask, None, iterations=2)
            # 膨胀操作，先腐蚀后膨胀以滤除噪声
            mask = cv2.dilate(mask, None, iterations=2)
            close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
            cv2.imshow('mask', mask)

            # 寻找图中轮廓
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            # 如果存在至少一个轮廓则进行如下操作
            if len(cnts) > 0:
                # 找到面积最大的轮廓
                c = max(cnts, key=cv2.contourArea)
                # 使用最小外接圆圈出面积最大的轮廓
                ((x, y), radius) = cv2.minEnclosingCircle(c)

                print('光斑位置：', x, y)
                # 计算轮廓的矩
                M = cv2.moments(c)
                # 计算轮廓的重心
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  # 通过矩计算中心xy坐标

                # 只处理尺寸足够大的轮廓
                if radius > 5:
                    # 画出最小外接圆
                    cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    # 画出重心
                    cv2.circle(image, center, 2, (0, 0, 255), -1)

            # 显示处理后的帧
            cv2.imshow('Camera', image)

            # 检测键盘按键，按下'q'键退出循环
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass

    # 释放摄像头资源
    HaiKang.close_device(cam, data_buf)
    cv2.destroyAllWindows()
    print("\n程序结束。")


if __name__ == "__main__":
    adjust_red_thresholds()
