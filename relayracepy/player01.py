# -*- encoding: UTF-8 -*-
import argparse

import cv2
import numpy as np
import math
from PIL import Image
import vision_definitions
import time

from naoqi import ALProxy


class MyClass():
    def __init__(self, robotip, port=9559):
        self.motionProxy = ALProxy("ALMotion", robotip, port)
        self.postureProxy = ALProxy("ALRobotPosture", robotip, port)
        self.camProxy = ALProxy("ALVideoDevice", robotip, port)

    def onLoad(self):
        # put initialization code here
        pass

    def onUnload(self):
        pass

    def onInput_onStart(self):
        i = 0
        # 步态设置
        maxstepx = 0.04
        maxstepy = 0.14
        maxsteptheta = 0.4
        maxstepfrequency = 0.5
        stepheight = 0.02
        torsowx = 0.0
        torsowy = 0.0
        moveConfig = [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]]

        self.postureProxy.goToPosture("StandInit", 0.5)
        self.motionProxy.moveInit()
        # self.motionProxy.angleInterpolation('HeadPitch',10*math.pi/180.0,0.8,False)
        self.motionProxy.setAngles('HeadPitch', 10 * math.pi / 180.0, 0.8)
        # almotion_init_setAngles()
        # almotion_init_setAngles()
        self.camProxy.setActiveCamera(1)  # 0 is up camera
        # Register a Generic Video Module
        resolution = vision_definitions.kQVGA
        colorSpace = vision_definitions.kBGRColorSpace
        fps = 30

        nameId = self.camProxy.subscribe("python_GVM", resolution, colorSpace, fps)
        # 设置曝光度模式
        self.camProxy.setCamerasParameter(nameId, 22, 2)
        print 'getting images in remote'

        x = []
        y = []
        stiffness = 1
        # f=open(r'tt.txt','a')
        robotPosition = self.motionProxy.getRobotPosition(0)
        n = robotPosition[2]
        currentPosition = [0, 0, 0]

        cv2.namedWindow('Video')
        cv2.namedWindow('mask1')
        cv2.namedWindow('mask')
        cv2.namedWindow('result')

        while (1):
            # 获取机器人位置坐标，计算里程
            robotNewPosition = self.motionProxy.getRobotPosition(0)
            currentPosition[0] = ((robotNewPosition[0] - robotPosition[0]) * 100) * math.cos(n) + ((
                                                                                                       robotNewPosition[
                                                                                                           1] -
                                                                                                       robotPosition[
                                                                                                           1]) * 100) * math.sin(
                n)
            currentPosition[1] = ((robotNewPosition[1] - robotPosition[1]) * 100) * math.cos(n) - ((
                                                                                                       robotNewPosition[
                                                                                                           0] -
                                                                                                       robotPosition[
                                                                                                           0]) * 100) * math.sin(
                n)
            currentPosition[2] = (robotNewPosition[2] - robotPosition[2]) * 57.32
            # f.write(str(currentPosition)+'\n')
            # f.close
            x.append(currentPosition[0])
            y.append(currentPosition[1])
            print"currentPosition is :"
            print currentPosition
            ##########################
            i = i + 1
            print "getting image " + str(i)
            naoImage = self.camProxy.getImageRemote(nameId)

            imageWidth = naoImage[0]
            imageHeight = naoImage[1]
            array = naoImage[6]

            # im = Image.fromstring("RGB", (imageWidth, imageHeight), array)
            im = Image.frombytes("RGB", (imageWidth, imageHeight), array)
            # -----------
            frame = np.asarray(im)
            cv2.imshow('Video', frame)
            # 对读取的图像矩阵进行计算，计算白线位置
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lowera = np.array([0, 0, 221])
            uppera = np.array([180, 30, 255])
            mask1 = cv2.inRange(hsv, lowera, uppera)
            cv2.imshow('mask1', mask1)
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            cv2.imshow('mask', mask)

            newlines = cv2.HoughLines(mask, 1, np.pi / 180, 80)
            if newlines == None:
                print "NO LINE"
                newlines1 = [[0, 0]]
            else:
                newlines1 = newlines[:, 0, :]
                # newlines1 = newlines[0]
                print "newlines1", newlines1[0]
                # print "point line:",newlines1[0]
                for rho, theta in newlines1[:]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    print "a,b:-------------"
                    print a, b
                    x0 = a * rho
                    y0 = b * rho
                    print "x0,y0:"
                    print x0, y0
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))
                    print x1, y1
                    print x2, y2
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.imshow('result', frame)
            single_line_value = newlines1[0]
            # single_line_value = CalculateCenterOfWhiteLine(frame)
            # print "single_line_value:",single_line_value

            if single_line_value[0] != 0:
                # motionProxy.move(0.1,0,0,moveConfig)
                if single_line_value[1] <= 1.04:
                    print "turn right"
                    # 右转前进
                    self.motionProxy.move(0.3, 0, -0.2, moveConfig)
                elif single_line_value[1] >= 2.09:
                    print "turn left"
                    # 左转前进
                    self.motionProxy.move(0.3, 0, 0.2, moveConfig)
                else:
                    print "mindle"
                    # 识别到终点直线
                    if single_line_value[1] <= 1.57 and single_line_value[1] > 1.04:
                        self.motionProxy.move(0.3, 0, 0.2, moveConfig)
                    if single_line_value[1] <= 2.09 and single_line_value[1] > 1.57:
                        self.motionProxy.move(0.3, 0, -0.2, moveConfig)
                        # motionProxy.move(0.3,0.0,0.0,moveConfig)
            else:
                print "no line"
                if currentPosition[0] <= 310:
                    self.motionProxy.move(0.1, 0.0, 0.0, moveConfig)
                else:
                    self.motionProxy.move(0.0, 0.0, 0.0, moveConfig)
                    self.motionProxy.rest()
                    break
            cv2.waitKey(10)

        self.camProxy.unsubscribe(nameId)
        print "camProxy has unsubscribe"
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    my = MyClass(args.ip, args.port)
    my.onInput_onStart()
