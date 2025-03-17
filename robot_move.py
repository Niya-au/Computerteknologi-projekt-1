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

        #distance at which the robot stops from an object
        self.stop_distance = 0.5
        #works with angular velocity
        self.tele_twist = Twist()
        #intial/base velocity
        self.tele_twist.linear.x = 0.1
        #initial/base angular velocity
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
   
    obstacle_distance_left = min(self.scan_ranges[0:left_range])
    obstacle_distance_front_left = min(self.scan_ranges[left_range:front_left_range])
    obstacle_distance_front = min(self.scan_ranges[front_left_range:front_right_range])
    obstacle_distance_front_right = min(self.scan_ranges[front_right_range:right_range])
    obstacle_distance_right = min(self.scan_ranges[right_range:360])
      

        twist = Twist()
        #when it detects a distance self.stop_distance from the object
        if obstacle_distance < self.stop_distance:
            #Moves forward with speed 0.1 m/s
            twist.linear.x = 0.1
            self.get_logger().info('Obstacle detected! Stopping.', throttle_duration_sec=2)
            #turns to the anticlockwise using angular speed 0.5 rad/s
            twist.angular.z = 0.5

            #detects a distance 0.4 meters from object
            if 0.0 < obstacle_distance < 0.40:
                #stops rotating
                twist.angular.z = 0.0
                #moves backwards by speed -0.1 m/s
                twist.linear.x = -0.1

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
