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
        #Finds distance from left, right, front, front-left, front-right to an object 

        #ranges given as angles in degrees. Total range is 90 (most right) to -90 (most left) where 0 is center/front 
        #The self_scan_ranges function returns an array of values. Using min function, the minimum value is taken from the returned data array
        obstacle_distance_left = min(self.scan_ranges[-90:-72])
        
        obstacle_distance_front_left = min(self.scan_ranges[-72:-36])
       
       #front interval
       #-36 to -1 range used as the robot has a hard time with transition from positive-negative angle range
        obstacle_distance_front_1 = min(self.scan_ranges[-36:-1])
        obstacle_distance_front_2 = min(self.scan_ranges[0:36])
        obstacle_distance_front = obstacle_distance_front_1 + obstacle_distance_front_2
        
        obstacle_distance_front_right = min(self.scan_ranges[36:72])
        
        obstacle_distance_right = min(self.scan_ranges[72:90])
       

        twist = Twist()

        #the stop distance for front needed to be smaller than the others
        if obstacle_distance_front < 0.6:
           twist.linear.x = 0.0
           self.get_logger().info('Obstacle detected! Stopping.', throttle_duration_sec=2)
           twist.linear.x = -0.1
           twist.angular.z = 0.5

        #robot turns to the right when an object is detected to the left
        elif obstacle_distance_left < self.stop_distance:
            twist.linear.x = 0.1
            twist.angular.z = 0.5

        #robot turns to the right when an object is detected to the front-left
        elif obstacle_distance_front_left < self.stop_distance:
            twist.linear.x = 0.1
            #lower angular speed used for sharper turn (note: for precision try to make it slower)
            twist.angular.z = 0.25

        #robot turns to the left when object is detected to the front-right
        elif obstacle_distance_front_right < self.stop_distance:
            twist.linear.x = 0.1
            twist.angular.z = -0.25

        #robot turns to the left when an object is detected to the right
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
