import cv2
import numpy as np
import struct
#import serial
# import time

import sys

import HaiKang

current_dir = sys.path[0]
sys.path.insert(0, current_dir)
sys.path.append(r".\MvImport")
from MvCameraControl_class import *

# 配置串口
#ser = serial.Serial('/dev/ttyUSB0', 754000)

# -----------找到红色光斑的位置----------------


class NANO_RecData:
    def __init__(self):
        self.X_AIM = 0
        self.Y_AIM = 0
        self.X_CURRENT = 0
        self.Y_CURRENT = 0
        self.NOT_FIND_AIM = 0


# 初始化数据
NANO_rec_data = NANO_RecData( )


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

    # ---------白纸 白板上的绿色激光阈值
    # lower_red = np.array([35, 100, 100])
    # upper_red = np.array([85, 255, 255])

    # lower_red = np.array([60, 16, 146])
    # upper_red = np.array([85, 49, 219])

    # 对图片进行二值化处理
    mask = cv2.inRange(hsv, lower_red, upper_red)
    # 腐蚀操作
    mask = cv2.erode(mask, None, iterations=2)
    # 膨胀操作，先腐蚀后膨胀以滤除噪声
    mask = cv2.dilate(mask, None, iterations=5)
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

    return x, y


# ---------绘制需要的边缘（四边形）到空白图像上，并且返回边缘的四个点-----------
def find_Contours(original_image, img):
    co = []
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # 遍历边缘，仅绘制满足面积条件的边缘
    for contour in contours:
        # 找到面积最大的轮廓
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

def find_contours(original_image, img):
    co = []
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # 遍历边缘，仅绘制满足面积条件的边缘
    for contour in contours:
        # 找到面积最大的轮廓
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
                cv2.circle(original_image, (x, y), 5, (0, 0, 255), -1)
    return co

# dian是一个{（a,b),(c,d),...(e,f) }这样的数据类型
# 顺序为 左上 左下 右下 右上
# get_distance用来获得所有点之间的距离，并且不重复计算
def get_distance(dian):
    distances = []
    for i in range(len(dian)):
        for j in range(i + 1, len(dian)):
            x1, y1 = dian [i]
            x2, y2 = dian [j]
            distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            distances.append(distance)
    return distances


# 生成两点之间线上的若干点
def Get_linspa(data1, data2):
    X = []
    Y = []
    # 将两个点作为数据点，创建 NumPy 数组
    x_data = np.array([data1 [0], data2 [0]])
    y_data = np.array([data1 [1], data2 [1]])
    # 使用 np.polyfit 进行线性回归计算
    coefficients = np.polyfit(x_data, y_data, 1)
    # 得到拟合直线的斜率和截距
    slope = coefficients [0]
    intercept = coefficients [1]
    # 使用斜率和截距来求出直线上的其他点的坐标
    x_values_top = np.linspace(start=data1 [0], stop=data2 [0], num=2)
    y_values_top = slope * x_values_top + intercept
    # 输出拟合直线的斜率和截距
    print(x_values_top)
    print("斜率:", slope)
    print("截距:", intercept)
    # 输出拟合直线上的其他点的坐标
    print("拟合直线上的点坐标：")
    for x, y in zip(x_values_top, y_values_top):
        print("({:.2f}, {:.2f})".format(x, y))
        X.append(x)
        Y.append(y)

    return X, Y, slope


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
    targets.extend(generate_edge_points(big [0], big [1], 8))
    targets.append(tuple(big [1]))

    # 生成右上角到右下角的等分点
    targets.extend(generate_edge_points(big [1], big [2], 8))
    targets.append(tuple(big [2]))

    # 生成右下角到左下角的等分点
    targets.extend(generate_edge_points(big [2], big [3], 8))
    targets.append(tuple(big [3]))

    # 生成左下角到左上角的等分点
    targets.extend(generate_edge_points(big [3], big [0], 8))

    return targets


#----------得黑色胶布内外两组的四个边缘点以及端点中心点---------
def control_system(big_, small_):
    # 得到内外矩形框四角坐标
    left_top1 = big_ [0]  # 外边框
    left_down1 = big_ [1]
    right_down1 = big_ [2]
    right_top1 = big_ [3]

    left_top2 = small_ [0]  # 内边框
    left_down2 = small_ [1]
    right_down2 = small_ [2]
    right_top2 = small_ [3]

    left_top = (left_top2 + left_top1) / 2  # 两框之间位置的四个角
    left_down = (left_down2 + left_down1) / 2
    right_down = (right_down2 + right_down1) / 2
    right_top = (right_top2 + right_top1) / 2

    # 拟合黑线之间的四条边  得到的xy和斜率
    left_outx, left_outy, left_slope = Get_linspa(left_top, left_down)
    down_outx, down_outy, down_slope = Get_linspa(left_down, right_down)
    right_outx, right_outy, right_slope = Get_linspa(right_down, right_top)
    right_outx.reverse( )  # 逆排序
    right_outy.reverse( )
    top_outx, top_outy, top_slope = Get_linspa(right_top, left_top)
    # print(left_outx, left_outy)
    # print(down_outx, down_outy)
    # print(right_outx, right_outy)
    # print(top_outx, top_outy)

    return left_top, left_down, right_down, right_top


# --------判断点 (x1, y1) 是否在点 (x2, y2)附近--------------
def is_near_center(x1, y1, x2, y2, threshold=20):
    distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return distance <= threshold


def Canny_detect():
    out_d = []
    flag_findbleak = 1
    target_index = 0
    targets = []

    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    # ch: 枚举设备 | en:Enum device
    # nTLayerType[IN] 枚举传输层 ，pstDevList[OUT] 设备列表
    HaiKang.Enum_device(tlayerType, deviceList)

    # 获取相机和图像数据缓存区
    cam, data_buf, nPayloadSize = HaiKang.enable_device(0, deviceList)  # 选择第一个设备

    while True:
        image = HaiKang.get_image(cam, data_buf, nPayloadSize)

        if flag_findbleak == 1:
            # -------边缘提取
            # 高斯滤波 Canny边缘算子
            img = cv2.GaussianBlur(image, (3, 3), 0)  # 用高斯平滑处理原图像降噪。
            canny = cv2.Canny(img, threshold1=53, threshold2=64, L2gradient=True)  # 得到边缘

            # 查找轮廓
            # 绘制满足面积条件的边缘

            cunt1 = 0  # 判断是大还是小边缘

            # 大边缘
            edge_img1 = np.zeros_like(canny)  # 创建空白图像
            # big_=[] big是外边缘
            big_ = find_Contours(img, canny)  # 外边缘
            print('大---', big_)  # 左上 左下 右下 右上

            # 小边缘
            cunt2 = 1
            # 删除外边缘节点
            for point in big_:
                cv2.circle(canny, tuple(point), 10, (0, 0, 0), -1)
            # small_[]
            small_ = find_contours(img, canny)  # 再找一个方框（内边缘）
            print('小----', small_)  # 左上 左下 右下 右上

            # 计算每个点之间的距离
            big_dis = get_distance(big_)
            small_dis = get_distance(small_)

            # 计算距离用于判断检测点位是否正确
            print("每个点之间的距离：")
            print(big_dis)
            print(small_dis)

            if (0 < len(big_dis)< 200) and (0 < len(small_dis) < 200):
                if big_dis != small_dis:
                    # out_d[] in_d[] 是检测正确之后的边缘四点
                    out_d = big_
                    in_d = small_
                    flag_findbleak = 0
                    left_top, left_down, right_down, right_top = control_system(out_d, in_d)
                    targetsTmp = [left_top, right_top, right_down, left_down]
                    targets = expand_targets(targetsTmp)
                    # cv2.imwrite('photo.jpg', img)

            # cv2.imshow("canny", canny)
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
                if is_near_center(rx, ry, target_x, target_y):
                    print("红光点在目标点附近")
                    target_index = (target_index + 1) % len(targets)
                    print("切换到下一个目标点:", targets[target_index])
                    NANO_rec_data.NOT_FIND_AIM = 0
                else:
                    print("红光点不在目标点附近")
                    print("目标点：", targets[target_index])
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
            else:
                NANO_rec_data.NOT_FIND_AIM = 0

            # 绘制矩形框和其他信息
            if len(out_d) != 0 & len(in_d) != 0:
                for i in range(len(out_d)):
                    cv2.line(image, tuple(out_d[i]), tuple(out_d[(i + 1) % len(out_d)]), (0, 255, 0), 2)
                for i in range(len(out_d)):
                    cv2.line(image, tuple(in_d[i]), tuple(in_d[(i + 1) % len(in_d)]), (0, 0, 255), 2)
            # 打包数据并发送
            data = struct.pack('iiiii', NANO_rec_data.X_AIM, NANO_rec_data.Y_AIM, NANO_rec_data.X_CURRENT,
                               NANO_rec_data.Y_CURRENT, NANO_rec_data.NOT_FIND_AIM)
            # ser.write(data)
            cv2.imshow('frame', image)

            # esc键退出（设置读帧间隔时间）
        c = cv2.waitKey(5)
        if c == 27:  # ESC
            break


if __name__ == "__main__":
    i = 0
    Canny_detect( )
    cv2.waitKey(0)
    cv2.destroyAllWindows( )
