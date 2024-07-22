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
ser = serial.Serial('/dev/ttyUSB0', 115200)

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

    # ---------白纸 白板上的绿色激光阈值
    #lower_green = np.array([0, 0, 83])
    #upper_green = np.array([179, 255, 255])

    lower_red = np.array([0, 0, 87])
    upper_red = np.array([179, 255, 255])

    # 对图片进行二值化处理
    mask = cv2.inRange(hsv, lower_red, upper_red)
    # 腐蚀操作
    mask = cv2.erode(mask, None, iterations=2)
    # 膨胀操作，先腐蚀后膨胀以滤除噪声
    mask = cv2.dilate(mask, None, iterations=2)
    # cv2.imshow('mask', mask)

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

#--------判断点 (x1, y1) 是否在点 (x2, y2)附近--------------。
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
    x_values_top = np.linspace(start=data1[0], stop=data2[0], num=1)
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


# ---------------得到四个边缘点和矩形中心点------------------------
def control_system(big_, rx, ry, state=0):
    # 得到矩形框四角坐标
    left_top1 = big_[0]  # 外边框
    left_down1 = big_[1]
    right_down1 = big_[2]
    right_top1 = big_[3]

    # 拟合黑线之间的四条边  得到的xy和斜率
    left_outx, left_outy, left_slope = Get_linspa(left_top1, left_down1)
    down_outx, down_outy, down_slope = Get_linspa(left_down1, right_down1)
    right_outx, right_outy, right_slope = Get_linspa(right_down1, right_top1)
    right_outx.reverse()  # 逆排序
    right_outy.reverse()
    top_outx, top_outy, top_slope = Get_linspa(right_top1, left_top1)
    # print(left_outx, left_outy)
    # print(down_outx, down_outy)
    # print(right_outx, right_outy)
    # print(top_outx, top_outy)

    # 计算四边形中心点
    center_x = (left_top1[0] + right_down1[0]) // 2
    center_y = (left_top1[1] + right_down1[1]) // 2

    return left_top1, left_down1, right_down1, right_top1, left_slope, down_slope, right_slope, top_slope, center_x, center_y


def Canny_detect():
    out_d = []
    flag_findbleak = 1
    # 获得设备信息
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    # ch: 枚举设备 | en:Enum device
    # nTLayerType[IN] 枚举传输层 ，pstDevList[OUT] 设备列表
    HaiKang.Enum_device(tlayerType, deviceList)

    # 获取相机和图像数据缓存区
    cam, data_buf, nPayloadSize = HaiKang.enable_device(0, deviceList)  # 选择第一个设备

    while (True):
        # 读取摄像头画面
        image = HaiKang.get_image(cam, data_buf, nPayloadSize)

        # 检测矩形框
        if flag_findbleak == 1:
            # -------边缘提取
            # 3、高斯滤波 Canny边缘算子
            img = cv2.GaussianBlur(image, (3, 3), 0)  # 用高斯平滑处理原图像降噪。
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            canny = cv2.Canny(gray, threshold1=22, threshold2=64, L2gradient=True)  # 调整Canny边缘检测的参数

            # 应用闭运算
            kernel = np.ones((3, 3), np.uint8)
            closed = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)

            # 显示结果
            # cv2.imshow('Canny Edge Detection', canny)
            # cv2.imshow('Closed Edge Detection', closed)

            # 4. 查找轮廓
            # 方法1 5. 绘制满足面积条件的边缘
            edge_img1 = np.zeros_like(closed)  # 创建空白图像
            # big是外边缘
            big_ = find_largest_contour(img, closed)  # 外边缘
            print('大---', big_)  # 左上 左下 右下 右上

            for i, point in enumerate(big_):
                # 确保 point 是一个包含两个整数的元组或列表
                print(f"Processing point: {point}")
                cv2.putText(img, f"Point {i + 1}: ({point[0]}, {point[1]})", tuple(point), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)
            if len(big_) > 0 and cv2.contourArea(big_) > 100000:
                out_d = big_
                flag_findbleak = 0
            cv2.imshow("img", img)

        # 开始检测光斑
        elif flag_findbleak == 0:
            img = cv2.GaussianBlur(image, (3, 3), 0)  # 用高斯平滑处理原图像降噪。
            # 光斑的xy坐标
            red_x, red_y = Find_red(img)
            # 黑框里面的四个角坐标和中心点
            left_top, left_down, right_down, right_top, left_slope, down_slope, right_slope, top_slope, center_x, center_y = control_system(
                out_d, red_x, red_y, 0)
            # cv2.imshow("find_red", img)
            if red_x != 0 and red_y != 0:
                # 判断光斑是否在中心点附近
                if is_near_center(red_x, red_y, center_x, center_y, threshold=10):
                    print("光斑已在中心点附近，停止发送坐标")
                    NANO_rec_data.NOT_FIND_AIM = 1
                else:
                    # 计算当前点和中心点之间的目标点
                    target_x, target_y, _ = Get_linspa((center_x, center_y), (red_x, red_y))
                    for x, y in zip(target_x, target_y):
                        print("目标点: ({:.2f}, {:.2f})".format(x, y))

                    # 绘制当前点和目标点
                    cv2.circle(image, (int(red_x), int(red_y)), 5, (0, 0, 255), -1)  # 绘制光斑点
                    cv2.circle(image, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)  # 绘制中心点
                    for x, y in zip(target_x, target_y):
                        cv2.circle(image, (int(x), int(y)), 3, (255, 0, 0), -1)  # 绘制目标点

                    # 更新数据结构
                    NANO_rec_data.X_AIM = int(target_x[-1])
                    NANO_rec_data.Y_AIM = int(target_y[-1])
                    NANO_rec_data.X_CURRENT = int(red_x)
                    NANO_rec_data.Y_CURRENT = int(red_y)
                    NANO_rec_data.NOT_FIND_AIM = 0
            else:
                NANO_rec_data.NOT_FIND_AIM = 1

            # 打包数据并发送
            data = struct.pack('iiiii', NANO_rec_data.X_AIM, NANO_rec_data.Y_AIM, NANO_rec_data.X_CURRENT,
                               NANO_rec_data.Y_CURRENT, NANO_rec_data.NOT_FIND_AIM)
            ser.write(data)

            cv2.imshow('frame', image)

    # esc键退出（设置读帧间隔时间）
        c = cv2.waitKey(5)
        if c == 27:  # ESC
            break






if __name__ == "__main__":
    Canny_detect()
    cv2.waitKey(0)
    cv2.destroyAllWindows()
