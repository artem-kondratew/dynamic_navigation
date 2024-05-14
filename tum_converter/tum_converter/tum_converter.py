#!/usr/bin/env python3

import sys

import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import CameraInfo, Image


class TumConverter(Node):

    def __init__(self):
        super().__init__('tum_converter')

        self.rgb_pub_ = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub_ = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.info_pub_ = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.bridge_ = CvBridge()
        self.timer_ = self.create_timer(1 / 30, self.callback)
        self.cnt = 0

        self.rgb_msgs = []
        self.info_msgs = []
        self.depth_msgs = []

        self.read_tum()

        print(f'{len(self.rgb_msgs)} pairs of images converted')

        print('node started')

    def callback(self):
        if self.cnt < len(self.rgb_msgs):
            self.rgb_pub_.publish(self.rgb_msgs[self.cnt])
            self.info_pub_.publish(self.info_msgs[self.cnt])
            self.depth_pub_.publish(self.depth_msgs[self.cnt])
            self.cnt += 1
        else:
            print('node finished')
            exit(0)

    def read_file_list(self, filename):
        file = open(filename)
        data = file.read()
        lines = data.replace(","," ").replace("\t"," ").split("\n") 
        list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
        list = [(float(l[0]),l[1:]) for l in list if len(l)>1]
        return dict(list)

    def associate(self, first_list : dict, second_list : dict, offset,max_difference):
        first_keys = list(first_list)
        second_keys = list(second_list)
        potential_matches = [(abs(a - (b + offset)), a, b) 
                             for a in first_keys 
                             for b in second_keys 
                             if abs(a - (b + offset)) < max_difference]
        potential_matches.sort()
        matches = []
        for diff, a, b in potential_matches:
            if a in first_keys and b in second_keys:
                first_keys.remove(a)
                second_keys.remove(b)
                matches.append((a, b))

        matches.sort()
        return matches

    def get_tuples(self, p1, p2):
        first_list = self.read_file_list(p1)
        second_list = self.read_file_list(p2)

        matches = self.associate(first_list, second_list,float(0.0),float(0.02))    

        tuples = []

        for a,b in matches:
            tuples.append((first_list[a], second_list[b]))
        return tuples
        
    def read_tum(self):
        if len(sys.argv) != 2:
            print('usage: ros2 run tum_converter <src_folder>')
            exit(1)
        src = sys.argv[1]

        bridge = CvBridge()

        tuples = self.get_tuples(f'{src}/rgb.txt', f'{src}/depth.txt')
        print(f'preparing to convert {len(tuples)} tuples of images')

        for t in tuples:
            if len(t) != 2:
                continue
            rgb = cv.imread(src + '/' + t[0][0])
            # print(t[0][0].split('.'))
            rgb_msg : Image = bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
            rgb_msg.header.stamp.sec = int(t[0][0].split('.')[0][4:])
            rgb_msg.header.stamp.nanosec = int(t[0][0].split('.')[1])
            # print(rgb_msg.header.stamp.sec, rgb_msg.header.stamp.nanosec)
            rgb_msg.header.frame_id = 'openni_rgb_optical_frame'
            self.rgb_msgs.append(rgb_msg)

            depth = cv.imread(src + '/' + t[1][0], cv.IMREAD_ANYDEPTH)
            depth_msg : Image = bridge.cv2_to_imgmsg(depth, encoding='16UC1')
            depth_msg.header.stamp = rgb_msg.header.stamp
            depth_msg.header.frame_id = 'openni_rgb_optical_frame'
            self.depth_msgs.append(depth_msg)

            info = CameraInfo()
            info.header = rgb_msg.header
            info.width = rgb_msg.width
            info.height = rgb_msg.height
            info.distortion_model = 'plumb_bob'
            info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            info.k = [525.0, 0.0, 319.5,
                      0.0, 525.0, 239.5,
                      0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [535.43310546875, 0.0, 320.106652814575, 0.0, 0.0, 539.212524414062, 247.632132204719, 0.0, 0.0, 0.0, 1.0, 0.0]
            info.roi.do_rectify = False
            self.info_msgs.append(info)


def main(args=None):
    rclpy.init(args=args)
    motion_detector = TumConverter()
    rclpy.spin(motion_detector)
    motion_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
