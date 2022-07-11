#!/usr/bin/env python3
# MIT License
# Copyright (c) 2019-2022 JetsonHacks

# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from copy import deepcopy

from ament_index_python.resources import get_resource

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Image


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=480,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


class CameraPublisher(Node):

    def __init__(self):
        self.video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        if not self.video_capture.isOpened():
            return

        super().__init__('image_publisher')
        self.i = 0
        qos_profile = QoSProfile(depth=1)

        self.pub = self.create_publisher(Image, '/camera', qos_profile=qos_profile)
        timer_period = 0.1
        self.tmr = self.create_timer(timer_period, self.timer_callback)

        ret_val, img = self.video_capture.read()

        self.msg = Image()
        self.msg.data = [int(b) for b in list(img.flatten())]
        self.msg.height = img.shape[0]
        self.msg.width = img.shape[1]
        self.msg.encoding = 'rgb8'
        self.msg.step = img.shape[1] * img.shape[2]

    def timer_callback(self):
        self.i += 1
        self.get_logger().info('Publishing Camera capture: "{0}"'.format(self.i))
        ret_val, img = self.video_capture.read()

        self.msg.data = [int(b) for b in list(img.flatten())]
        self.pub.publish(self.msg)


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = CameraPublisher()

    rclpy.spin(node)

    video_capture.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
