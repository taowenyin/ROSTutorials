#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError


class MotionDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # 创建cv_bridge
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(
            "cv_bridge_image", Image, queue_size=1)

        # 设置参数：最小区域、阈值
        self.minArea = rospy.get_param("~minArea", 500)
        self.threshold = rospy.get_param("~threshold", 25)
        # 第一帧
        self.firstFrame = None
        self.text = "Unoccupied"

        # 初始化定位rgb图像数据的订阅者
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
        try:
            # 使用cv_bridge把ROS图像转化为OpenCV的图像格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # 获取图像的信息
            frame = np.array(cv_image, dtype="uint8")
        except CvBridgeError, e:
            print(e)

        # 创建灰度图像
        grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 高斯滤波
        grey_image = cv2.GaussianBlur(grey_image, (21, 21), 0)

        # 如果没有保存第一帧图片，那么就保存
        if self.firstFrame is None:
            self.firstFrame = grey_image
            return

        # 计算两个图片的差
        frameDelta = cv2.absdiff(self.firstFrame, grey_image)
        # 对图像进行二值化
        (retval, thresh) = cv2.threshold(frameDelta, self.threshold,
                                         255, cv2.THRESH_BINARY)
        # 对图像进行膨胀操作
        thresh = cv2.dilate(thresh, None, iterations=2)
        # 进行轮廓检测
        binary, cnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)

        for c in cnts:
            # 如果面积小于最小面积，那么就舍弃
            if cv2.contourArea(c) < self.minArea:
                continue

            # 获取一个矩形外框
            (x, y, w, h) = cv2.boundingRect(c)
            # 绘制一个矩形框
            cv2.rectangle(frame, (x, y), (x + w, y + h), (50, 255, 50), 2)
            self.text = "Occupied"

        # 在矩形框上绘制一个文字
        cv2.putText(frame, "Status: {}".format(self.text), (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 发布绘制好的图像数据
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def cleanup(self):
        print("Shutting down vision node.")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        # 初始化ros节点
        rospy.init_node("motion_detector")
        rospy.loginfo("motion_detector node is started...")
        rospy.loginfo("Please subscribe the ROS image.")
        MotionDetector()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down motion detector node."
        cv2.destroyAllWindows()
