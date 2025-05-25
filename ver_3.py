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
import time

class RGB_victim_Detection(Node):

   def __init__(self):
     self.led = LED(17)

 Get I2C bus
     self.bus = smbus.SMBus(1)

     self.bus.write_byte_data(0x44, 0x01, 0x05)

     self.time.sleep(1)

     print("Reading colour values and displaying them in a new window\n")

   def getAndUpdateColour(self):
       if True:
           self.data = self.bus.read_i2c_block_data(0x44, 0x09, 6) #Selects the right registers
           self.green = self.data[1] + self.data[0]/256 # Calculates the levels of each color [0, 255]
           self.red = self.data[3] + self.data[2]/256
           self.blue = self.data[5] + self. data[4]/256
 # Determines the color based on which has the higher value
           self.color = ""
           if self.green > self.red and self.green > self.blue:
              self.color = "Green"
           elif self.blue > self.red:
              self.color = "Blue"
           else:
              self.color = "Red"

           print("RGB(%d %d %d)" % (self.red, self.green, self.blue))

           print("The color is: " + self.color)

           if self.color == self.green or self.color == self.red or self.color == self.blue:
              self.led.on()
              self.sleep(1)
              self.led.on()
              self.sleep(1)


           #self.time.sleep(2)
         
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
        self.tele_twist.linear.x = 0.15
        self.tele_twist.angular.z = 0.0

        self.average_linear_speed = 0.0
        self.speed_updates = 0.0
        self.speed_accumulation = 0.0

        self.collision_counter = 0.0

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

    def get_average_speed(self):
        return self.speed_accumulation/self.speed_updates

    def get_collision_count(self):
        return self.collision_counter

     
    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def timer_callback(self):
        if self.has_scan_received:
            self.detect_obstacle()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        #publishes action
        self.cmd_vel_pub.publish(twist)

    def detect_obstacle(self):

        #Finds distance from left, right, front, front-left, front-right to an object

        #ranges given as angles in degrees. Total range is 90 (most right) to -90 (most left) where 0 is center/front
        #The self_scan_ranges function returns an array of values. Using min function, the minimum value is taken from >
        left_range = int(len(self.scan_ranges) / 5)
        front_left_range = int(len(self.scan_ranges) * 2 / 5)
        front_right_range = int(len(self.scan_ranges) * 3 / 5)
        right_range = int(len(self.scan_ranges) * 4 / 5)

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
            twist.linear.x = 0.10
            twist.angular.z = 0.5

        #robot turns to the right when an object is detected to the front-left
        elif obstacle_distance_front_left < self.stop_distance:
            twist.linear.x = 0.10
            twist.angular.z = 0.25

        #robot turns to the left when object is detected to the front-right
        elif obstacle_distance_front_right < self.stop_distance:
            twist.linear.x = 0.10
            twist.angular.z = -0.5


        else:
            twist = self.tele_twist

        #publish action
        self.cmd_vel_pub.publish(twist)

        #calculations for average speed calculation
        self.speed_updates = self.speed_updates + 1
        self.speed_accumulation = self.speed_accumulation + twist.linear.x

       #calculated collision count based
        if obstacle_distance_front < 0.17 or obstacle_distance_right < 0.20 or obstacle_distance_left < 0.20 or obstacl>           
           self.collision_counter = self.collision_counter + 1

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()

    #determines how long the program will run
    start_time = time.time()
    end_time = start_time + 30.0

    #within the specified time, the program will run
    while time.time() < end_time:
        rclpy.spin_once(turtlebot3_obstacle_detection, timeout_sec=0.1)

    #stops the motors after time runs out
    turtlebot3_obstacle_detection.stop_robot()

    #prints speed and collision count
    print("Average speed:", turtlebot3_obstacle_detection.get_average_speed())
    print("\nCollision count:", turtlebot3_obstacle_detection.get_collision_count())

    turtlebot3_obstacle_detection.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


