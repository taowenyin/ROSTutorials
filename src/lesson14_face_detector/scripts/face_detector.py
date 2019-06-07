#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2
import numpy as np


class FaceDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # 创建cv_bridge
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(
            "cv_bridge_image", Image, queue_size=1)

        # 获取haar特征的级联表的XML文件，文件路径在launch文件中配置
        cascade_1 = rospy.get_param("~cascade_1", "")
        cascade_2 = rospy.get_param("~cascade_2", "")

        # 使用级联表初始化haar特征检测器，载入文件路径前面不要加“file://”前缀
        self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        self.cascade_2 = cv2.CascadeClassifier(cascade_2)

        # 设置级联表的参数，优化人脸识别，参数在launch文件中配置
        self.haar_scaleFactor = rospy.get_param("~haar_scaleFactor", 1.2)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 2)
        self.haar_minSize = rospy.get_param("~haar_minSize", 40)
        self.haar_maxSize = rospy.get_param("~haar_maxSize", 60)

        self.color = (50, 255, 50)

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
        # 创建平衡直方图，减少光线影响
        grey_image = cv2.equalizeHist(grey_image)
        # 尝试检测人脸
        faces_result = self.detect_face(grey_image)

        # 在OpenCV的窗口中框出人脸区域
        if len(faces_result) > 0:
            for face in faces_result:
                x, y, w, h = face
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), self.color, 2)

        # 将识别后的图像转化为ROS消息并发布
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def detect_face(self, input_image):
        # 首先匹配正面人脸的模型
        if self.cascade_1:
            faces = self.cascade_1.detectMultiScale(
                input_image,
                self.haar_scaleFactor,
                self.haar_minNeighbors,
                cv2.CASCADE_SCALE_IMAGE,
                (self.haar_minSize, self.haar_maxSize))

        # 如果正面失败，那么就匹配侧面人脸模型
        if len(faces) == 0 and self.cascade_2:
            faces = self.cascade_2.detectMultiScale(
                input_image,
                self.haar_scaleFactor,
                self.haar_minNeighbors,
                cv2.CASCADE_SCALE_IMAGE,
                (self.haar_minSize, self.haar_maxSize))

        return faces

    def cleanup(self):
        print("Shutting down vision node.")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        rospy.init_node("face_detector")
        FaceDetector()
        rospy.loginfo("Face detector is started ...")
        rospy.loginfo("Please subscribe the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down face detector node.")
        cv2.destroyAllWindows()
