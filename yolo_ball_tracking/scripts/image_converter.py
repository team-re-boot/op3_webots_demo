#! /usr/bin/env python3

# Copyright 2024 Team Reboot.
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
# limitations under the License

import os
from launch.launch_context import LaunchContext
from launch_ros.substitutions import FindPackageShare
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from sensor_msgs.msg import Image

class ImageConverter(Node):
  def __init__(self):
    super().__init__('image_converter')

    self.bridge = CvBridge()

    qos_camera_data = qos_profile_sensor_data
    qos_camera_data.reliability = QoSReliabilityPolicy.RELIABLE
    self.image_subscriber = self.create_subscription(
      Image, '/robotis_op3/ROBOTIS_OP3/Camera/image_color',
      self.__on_camera_image,
      qos_camera_data
    )
    self.result_image_publisher = self.create_publisher(Image, "/convert/image", 10)

  def __on_camera_image(self, msg):
    try:
      img = self.bridge.imgmsg_to_cv2(msg)
    except CvBridgeError as e:
      print(e)

    rgb = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    img_msg = self.bridge.cv2_to_imgmsg(rgb, "bgr8")
    img_msg.header = msg.header

    self.result_image_publisher.publish(img_msg)


def main(args=None):
  rclpy.init(args=args)

  image_converter = ImageConverter()
  rclpy.spin(image_converter)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
