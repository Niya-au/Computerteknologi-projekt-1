#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
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
#
# Authors: Jeonggeun Lim, Ryan Shim, Gilbert

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan


class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('TurtleBot3 Obstacle Detection - Auto Move Enabled')
        print('----------------------------------------------')
        print('stop angle: -90 ~ 90 deg')
        print('stop distance: 0.5 m')
        print('----------------------------------------------')

        self.scan_ranges = []
        self.has_scan_received = False

        self.stop_distance = 0.25
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.1
        self.tele_twist.angular.z = 0.0

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

     def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def timer_callback(self):
        if self.has_scan_received:
            self.detect_obstacle()

    def detect_obstacle(self):
        left_range = int(len(self.scan_ranges) / 5)
        front_left_range = int(len(self.scan_ranges) * 2 / 5)
        front_right_range = int(len(self.scan_ranges) * 3 / 5)
        right_range = int(len(self.scan_ranges) * 4 / 5)

        obstacle_distance_left = min(self.scan_ranges[-90:-72])
        print("left distance is:", obstacle_distance_left)
        obstacle_distance_front_left = min(self.scan_ranges[-72:-36])
        print("left-front distance is:", obstacle_distance_front_left)
        obstacle_distance_front_1 = min(self.scan_ranges[-36:-1])
        obstacle_distance_front_2 = min(self.scan_ranges[0:36])
        obstacle_distance_front = obstacle_distance_front_1 + obstacle_distance_front_2
        print("front distance is:", obstacle_distance_front)
        obstacle_distance_front_right = min(self.scan_ranges[36:72])
        print("front-right distance is:", obstacle_distance_front_right)
        obstacle_distance_right = min(self.scan_ranges[72:90])
        print("right distance is:", obstacle_distance_right)

        twist = Twist()

        if obstacle_distance_front < 0.6:
           twist.linear.x = 0.0
           self.get_logger().info('Obstacle detected! Stopping.', throttle_duration_sec=2)
           twist.linear.x = -0.1
           twist.angular.z = 0.5

        elif obstacle_distance_left < self.stop_distance:
            twist.linear.x = 0.1
            twist.angular.z = 0.5


        elif obstacle_distance_front_left < self.stop_distance:
            twist.linear.x = 0.1
            twist.angular.z = 0.25

        elif obstacle_distance_front_right < self.stop_distance:
            twist.linear.x = 0.1
            twist.angular.z = -0.25

        elif obstacle_distance_right < self.stop_distance:
            twist.linear.x = 0.1
            twist.angular.z = -0.5


        else:
            twist = self.tele_twist

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
