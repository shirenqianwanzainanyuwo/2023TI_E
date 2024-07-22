import cv2
import numpy as np
import struct
import serial
import time


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
NANO_rec_data = NANO_RecData( )


# -----------找到红色光斑的位置----------------
def Find_red(frame):
    x = 0
    y = 0
    # 进行高斯模糊
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    # 转换颜色空间到HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    #lower_red = np.array([60, 16, 146])
    #upper_red = np.array([85, 49, 219])
    lower_red = np.array([0, 0, 193])
    upper_red = np.array([179, 255, 255])

    # 对图片进行二值化处理
    mask = cv2.inRange(hsv, lower_red, upper_red)
    # 腐蚀操作
    mask = cv2.erode(mask, None, iterations=2)
    # 膨胀操作，先腐蚀后膨胀以滤除噪声
    mask = cv2.dilate(mask, None, iterations=2)
    # 闭运算
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
    cv2.imshow('mask', mask)

    # 寻找图中轮廓
    cnts = cv2.findContours(mask.copy( ), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) [-2]
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
        center = (int(M ["m10"] / M ["m00"]), int(M ["m01"] / M ["m00"]))  # 通过矩计算中心xy坐标

        # 只处理尺寸足够大的轮廓
        if radius > 1:
            # 画出最小外接圆
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            # 画出重心
            cv2.circle(frame, center, 2, (0, 0, 255), -1)

    return int(x), int(y)


# ---------绘制需要的边缘（四边形）到空白图像上，并且返回边缘的四个点-----------
def find_largest_contour(original_image, img):
    co = []
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # 找到最大的轮廓
    if len(contours) == 0:
        return []

    c = max(contours, key=cv2.contourArea)
    # 对轮廓进行多边形近似，减少点数, approx返回多边形边数
    epsilon = 0.02 * cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, epsilon, True)

    # 确保近似后的轮廓是四边形
    if len(approx) == 4:
        # 获取矩形的四个角坐标
        co = approx.reshape(-1, 2)
        # 在原图上绘制四个角点
        for corner in co:
            x, y = corner
            cv2.circle(original_image, (x, y), 5, (0, 255, 0), -1)
    return co

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
    x_values_top = np.linspace(start=data1[0], stop=data2[0], num=2)
    y_values_top = slope * x_values_top + intercept
    # 输出拟合直线的斜率和截距
    # print(x_values_top)
    # print("斜率:", slope)
    # print("截距:", intercept)
    # 输出拟合直线上的其他点的坐标
    # print("拟合直线上的点坐标：")
    for x, y in zip(x_values_top, y_values_top):
        print("({:.2f}, {:.2f})".format(x, y))
        X.append(x)
        Y.append(y)

    return X, Y, slope

# --------判断点 (x1, y1) 是否在点 (x2, y2)附近--------------
def is_near_center(x1, y1, x2, y2, threshold=5):
    distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return distance <= threshold

#---------------------------------生成8分点--------------------------------
def expand_targets(big):
    targets = []

    # 辅助函数，用于生成等分点
    def generate_edge_points(start, end, num_points):
        x_values = np.linspace(start [0], end [0], num_points + 2) [1:-1]
        y_values = np.linspace(start [1], end [1], num_points + 2) [1:-1]
        return list(zip(x_values, y_values))

    # 生成左上角到右上角的等分点
    targets.append(tuple(big [0]))
    targets.extend(generate_edge_points(big [0], big [1], 12))
    targets.append(tuple(big [1]))

    # 生成右上角到右下角的等分点
    targets.extend(generate_edge_points(big [1], big [2], 12))
    targets.append(tuple(big [2]))

    # 生成右下角到左下角的等分点
    targets.extend(generate_edge_points(big [2], big [3], 12))
    targets.append(tuple(big [3]))

    # 生成左下角到左上角的等分点
    targets.extend(generate_edge_points(big [3], big [0], 12))

    return targets
# ---------------检测和控制系统主循环------------------------
def Canny_detect():
    out_d = []
    flag_findbleak = 1
    target_index = 0
    targets= []
    # 获得设备信息
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    # ch: 枚举设备 | en:Enum device
    # nTLayerType[IN] 枚举传输层 ，pstDevList[OUT] 设备列表
    HaiKang.Enum_device(tlayerType, deviceList)

    # 获取相机和图像数据缓存区
    cam, data_buf, nPayloadSize = HaiKang.enable_device(0, deviceList)  # 选择第一个设备

    while True:
        image = HaiKang.get_image(cam,data_buf, nPayloadSize)
        if flag_findbleak == 1:
            # 边缘提取
            img = cv2.GaussianBlur(image, (3, 3), 0)  # 用高斯平滑处理原图像降噪。
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            canny = cv2.Canny(gray, threshold1=22, threshold2=64, L2gradient=True)  # 调整Canny边缘检测的参数

            # 应用闭运算
            kernel = np.ones((3, 3), np.uint8)
            closed = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel, iterations=3)

            big = find_largest_contour(image, closed)
            print("big is ", big)

            for i, point in enumerate(big):
                # 确保 point 是一个包含两个整数的元组或列表
                print(f"Processing point: {point}")
                cv2.putText(img, f"Point {i + 1}: ({point[0]}, {point[1]})", tuple(point), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

            # 保存矩形框
            if len(big) > 0 and cv2.contourArea(big) > 100000:
                out_d = big
                flag_findbleak = 0
                targetstmp = [big[0], big[3], big[2], big[1]]  # 按顺时针顺序排列顶点
                targets = expand_targets(targetstmp)
                target_index = 0
                print("目标点序列:", targets)
            cv2.imshow("img", img)

        # 开始检测光斑
        else:
            img = cv2.GaussianBlur(image, (3, 3), 0)  # 用高斯平滑处理原图像降噪。
            # 识别红光点
            rx, ry = Find_red(img)

            # 目标点
            target_x, target_y = targets[target_index]

            if rx != 0 and ry != 0:
                # 判断红光点是否在目标点附近
                if is_near_center(rx, ry, target_x, target_y, threshold=20):
                    print("红光点在目标点附近")
                    target_index = (target_index + 1) % len(targets)
                    print("切换到下一个目标点:", targets[target_index])
                    NANO_rec_data.NOT_FIND_AIM = 0
                else:
                    print("红光点不在目标点附近")
                    # print("目标点：",targets [target_index])
                    # 计算当前点和中心点之间的目标点
                    targetcur_x, targetcur_y, _ = Get_linspa((target_x, target_y), (rx, ry))

                    for x, y in zip(targetcur_x, targetcur_y):
                        print("移动点: ({:.2f}, {:.2f})".format(x, y))
                    # 绘制当前点和目标点
                    cv2.circle(image, (int(rx), int(ry)), 5, (0, 0, 255), -1)  # 绘制光斑点
                    cv2.circle(image, (int(target_x), int(target_y)), 5, (0, 255, 0), -1)  # 绘制四点
                    for x, y in zip(targetcur_x, targetcur_y):
                        cv2.circle(image, (int(x), int(y)), 3, (255, 0, 0), -1)  # 绘制目标点

                    # 更新数据结构
                    NANO_rec_data.X_AIM = int(targetcur_x[0])
                    NANO_rec_data.Y_AIM = int(targetcur_y[0])
                    NANO_rec_data.X_CURRENT = int(rx)
                    NANO_rec_data.Y_CURRENT = int(ry)
                    NANO_rec_data.NOT_FIND_AIM = 0
            # else:
            # NANO_rec_data.NOT_FIND_AIM = 1314

            # 绘制矩形框和其他信息
            if len(out_d) != 0:
                for i in range(len(out_d)):
                    cv2.line(image, tuple(out_d[i]), tuple(out_d[(i + 1) % len(out_d)]), (0, 255, 0), 2)
            # 打包数据并发送
            data = struct.pack('iiiii', NANO_rec_data.X_AIM, NANO_rec_data.Y_AIM, NANO_rec_data.X_CURRENT,
                               NANO_rec_data.Y_CURRENT, NANO_rec_data.NOT_FIND_AIM)
            ser.write(data)
            # time.sleep(0.1)  # 延时0.1秒
            cv2.imshow('frame', image)

        # esc键退出（设置读帧间隔时间）
        c = cv2.waitKey(5)
        if c == 27:  # ESC
            break


if __name__ == "__main__":
    Canny_detect( )
    cv2.waitKey(0)
    cv2.destroyAllWindows( )
