#!/usr/bin/env python3


import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from rtabmap_msgs.msg import MotionDetectorData

from .submodules.yolo import Yolo
from .submodules.yolo_classes import classes_list


class MotionDetector(Node):

    def __init__(self):
        super().__init__('motion_detector')
        self.declare_parameters(namespace='', parameters=[('input_topic', ''),
                                                          ('camera_info_topic', ''),
                                                          ('rgb_topic', ''),
                                                          ('depth_topic', ''),
                                                          ('mask_topic', ''),
                                                          ('odom_topic', ''),
                                                          ('dynamic_detector_topic', ''),
                                                          ('create_mask', True)])
        self.create_mask = self.get_parameter('create_mask').value
        self.subscription_ = self.create_subscription(MotionDetectorData, self.get_parameter('input_topic').value, self.callback, 10)
        self.subscription_  # prevent unused variable warning
        self.camera_info_pub_ = self.create_publisher(CameraInfo, self.get_parameter('camera_info_topic').value, 10)
        self.rgb_pub_ = self.create_publisher(Image, self.get_parameter('rgb_topic').value, 10)
        self.depth_pub_ = self.create_publisher(Image, self.get_parameter('depth_topic').value, 10)
        self.mask_pub_ = self.create_publisher(Image, self.get_parameter('mask_topic').value, 10)
        self.odom_pub_ = self.create_publisher(Odometry, self.get_parameter('odom_topic').value, 10)
        self.dynamic_detector_pub_ = self.create_publisher(MotionDetectorData, self.get_parameter('dynamic_detector_topic').value, 10)
        self.bridge_ = CvBridge()
        self.kernel_ = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        self.model_ = Yolo(classes=classes_list)
        self.model_.run(np.zeros((480, 640, 3), dtype='uint8'))
        self.get_logger().info(f'create mask: {self.create_mask}')
        self.get_logger().info('model initialized')

    def get_boxes_coordinates(self, boxes):
        x = []
        for box in boxes:
            for v in box.xyxy.tolist()[0]:
                x.append(int(v))
        return x

    def callback(self, msg):
        cv_frame = self.bridge_.imgmsg_to_cv2(msg.rgb, desired_encoding='passthrough')
        depth = self.bridge_.imgmsg_to_cv2(msg.depth, desired_encoding='passthrough')
        depth_masked = depth.copy()
        boxes = []

        output = self.model_.run(cv_frame)
        success, mask = self.model_.merge_masks(output.masks)
        h, w = cv_frame.shape[:2]
        if not success or not self.create_mask:
            mask = np.zeros((h, w, 1), dtype='uint8')
            self.get_logger().info('no_masks')
        else:
            if self.model_.gpu:
                mask = mask[(w-h)//2:h+(w-h)//2, :]
            mask = cv.normalize(mask, None, 255, 0, cv.NORM_MINMAX, cv.CV_8U)
            mask = cv.dilate(mask, self.kernel_, iterations = 2)
            depth_masked[mask == 255] = 0
            boxes = self.get_boxes_coordinates(output.boxes)

        msg.mask = self.bridge_.cv2_to_imgmsg(mask, encoding='passthrough')
        msg.depth = self.bridge_.cv2_to_imgmsg(depth_masked, encoding='passthrough')
        msg.mask.header = msg.rgb.header
        msg.depth.header = msg.rgb.header
        self.camera_info_pub_.publish(msg.camera_info)
        self.rgb_pub_.publish(msg.rgb)
        self.depth_pub_.publish(msg.depth)
        self.mask_pub_.publish(msg.mask)
        self.odom_pub_.publish(msg.odom)

        msg.depth = self.bridge_.cv2_to_imgmsg(depth, encoding='passthrough')
        msg.boxes = boxes
        self.dynamic_detector_pub_.publish(msg)

        if False:
            cv.imshow('main_mask', mask)
            cv.imshow('frame', cv_frame)
            cv.imshow('depth', depth)
            cv.imshow('depth_masked', depth_masked)
            cv.waitKey(20)


def main(args=None):
    rclpy.init(args=args)
    
    motion_detector = MotionDetector()

    rclpy.spin(motion_detector)

    motion_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
