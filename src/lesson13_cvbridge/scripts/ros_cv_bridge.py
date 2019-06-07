#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2


class ImageConverter:
    def __init__(self):
        # 创建图像的发布者
        self.image_pub = rospy.Publisher("cv_bridge", Image, queue_size=1)
        # 创建CvBridge
        self.bridge = CvBridge()
        # 创建摄像头数据的接收者，以及接收到的回调函数
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            # 把接收到的图像消息数据转化为OpenCV的图像数据
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        # 获取图像的大小，并绘制一个圆
        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (60, 60), 30, (0, 0, 255), -1)

        # 显示一个图像
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)

        try:
            # 把绘制好的OpenCV图像在进行发布
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print e


if __name__ == '__main__':
    try:
        # 创建一个节点
        rospy.init_node("cv_bridge")
        rospy.loginfo("Starting cv_bridge node")
        # 接收、转换、发送图像
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down cv_bridge node")
        cv2.destroyAllWindows()
