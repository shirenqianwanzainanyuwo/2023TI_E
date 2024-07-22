import cv2
import numpy as np
import struct
import serial

import sys

import HaiKang

current_dir = sys.path[0]
sys.path.insert(0, current_dir)
sys.path.append(r".\MvImport")
from MvCameraControl_class import *
# 配置串口
ser = serial.Serial('/dev/ttyUSB0', 754000)

# 定义数据结构
class NANO_RecData:
    def __init__(self):
        self.X_AIM = 0
        self.Y_AIM = 0
        self.X_CURRENT = 0
        self.Y_CURRENT = 0
        self.NOT_FIND_AIM = 0

# 初始化数据
NANO_rec_data = NANO_RecData()

# -----------找到红色光斑的位置----------------
def Find_red(frame):
    x = 0
    y = 0
    # 进行高斯模糊
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    # 转换颜色空间到HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # ---------白纸 白板上的红色激光阈值
    lower_red = np.array([0, 0, 104])
    upper_red = np.array([179, 255, 255])

    # 对图片进行二值化处理
    mask = cv2.inRange(hsv, lower_red, upper_red)
    # 腐蚀操作
    mask = cv2.erode(mask, None, iterations=2)
    # 膨胀操作，先腐蚀后膨胀以滤除噪声
    mask = cv2.dilate(mask, None, iterations=2)
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
        if radius > 1:
            # 画出最小外接圆
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            # 画出重心
            cv2.circle(frame, center, 2, (0, 0, 255), -1)

    return x, y

# -----------找到绿色光斑的位置----------------
def Find_gre(frame):
    x = 0
    y = 0
    # 进行高斯模糊
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    # 转换颜色空间到HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # ---------白纸 白板上的绿色激光阈值
    lower_gre = np.array([0, 0, 83])
    upper_gre = np.array([179, 255, 255])

    # 对图片进行二值化处理
    mask = cv2.inRange(hsv, lower_gre, upper_gre)
    # 腐蚀操作
    mask = cv2.erode(mask, None, iterations=2)
    # 膨胀操作，先腐蚀后膨胀以滤除噪声
    mask = cv2.dilate(mask, None, iterations=2)
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
        if radius > 1:
            # 画出最小外接圆
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            # 画出重心
            cv2.circle(frame, center, 2, (0, 0, 255), -1)

    return x, y

#--------判断点 (x1, y1) 是否在点 (x2, y2)附近--------------
def is_near_center(x1, y1, x2, y2, threshold=5):
    distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return distance <= threshold

# --------------生成两点之间线上的若干点----------------------
def Get_linspa(data1, data2):
    X = []
    Y = []
    # 将两个点作为数据点，创建 NumPy 数组
    x_data = np.array([data1[0], data2[0]])
    y_data = np.array([data1[1], data2[1]])
    # 使用 np.polyfit 进行线性回归计算
    coefficients = np.polyfit(x_data, y_data, 1)
    # 得到拟合直线的斜率和截距
    slope = coefficients[0]
    intercept = coefficients[1]
    # 使用斜率和截距来求出直线上的其他点的坐标
    x_values_top = np.linspace(start=data1[0], stop=data2[0], num=10)
    y_values_top = slope * x_values_top + intercept
    # 输出拟合直线的斜率和截距
    for x, y in zip(x_values_top, y_values_top):
        print("({:.2f}, {:.2f})".format(x, y))
        X.append(x)
        Y.append(y)

    return X, Y, slope

def Canny_detect():
    '''
    整体的思路是：
    1. 检测红色光斑，获得红色光斑坐标，作为目标点。
    2. 检测绿色光斑，获得绿色光标坐标，作为当前点。
    '''

    flag_findbleak = 1

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

        # 检测红色光斑
        if flag_findbleak == 1:
            img = cv2.GaussianBlur(image, (3, 3), 0)  # 用高斯平滑处理原图像降噪。
            # 红色光斑的xy坐标
            red_x, red_y = Find_red(img)
            if red_x != 0 and red_y != 0:
                flag_findbleak = 0
            cv2.imshow("find_red", img)

        # 检测绿色光斑
        elif flag_findbleak == 0:
            img = cv2.GaussianBlur(image, (3, 3), 0)  # 用高斯平滑处理原图像降噪。
            # 绿色光斑的xy坐标
            gre_x, gre_y = Find_gre(img)
            cv2.imshow("find_gre", img)
            if gre_x != 0 and gre_y != 0:
                # 判断绿色光斑是否在红色光斑附近
                if is_near_center(red_x, red_y, gre_x, gre_y, threshold=10):
                    print("已经追随到目标")
                    NANO_rec_data.NOT_FIND_AIM = 0
                    flag_findbleak = 1  # 继续检测红色光斑

                else:
                    # 计算当前点和中心点之间的目标点
                    target_x, target_y, _ = Get_linspa((red_x, red_y), (gre_x, gre_y))
                    for x, y in zip(target_x, target_y):
                        print("目标点: ({:.2f}, {:.2f})".format(x, y))

                    # 绘制当前点和目标点
                    cv2.circle(image, (int(red_x), int(red_y)), 5, (0, 0, 255), -1)  # 绘制光斑点
                    cv2.circle(image, (int(gre_x), int(gre_y)), 5, (0, 255, 0), -1)  # 绘制中心点
                    for x, y in zip(target_x, target_y):
                        cv2.circle(image, (int(x), int(y)), 3, (255, 0, 0), -1)  # 绘制目标点

                    # 更新数据结构
                    NANO_rec_data.X_AIM = int(target_x[-1])
                    NANO_rec_data.Y_AIM = int(target_y[-1])
                    NANO_rec_data.X_CURRENT = int(gre_x)
                    NANO_rec_data.Y_CURRENT = int(gre_y)
                    NANO_rec_data.NOT_FIND_AIM = 0
            else:
                NANO_rec_data.NOT_FIND_AIM = 0

            # 打包数据并发送
            data = struct.pack('iiiii', NANO_rec_data.X_AIM, NANO_rec_data.Y_AIM, NANO_rec_data.X_CURRENT, NANO_rec_data.Y_CURRENT, NANO_rec_data.NOT_FIND_AIM)
            ser.write(data)
            cv2.imshow('frame', image)

        # esc键退出（设置读帧间隔时间）
        c = cv2.waitKey(5)
        if c == 27:  # ESC
            break

    HaiKang.close_device(cam, data_buf)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    Canny_detect()
