
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
# Authors: Jeonggeun Lim, Ryan Shim, Gilbert yaya

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import time
import smbus2 as smbus
from gpiozero import LED
from time import sleep
###############################################################################################################
############################################## VICTIM DETECTION ###############################################
###############################################################################################################
class RGBVictimDetection:
    def __init__(self):
        self.led = LED(17)
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(0x44, 0x01, 0x05)
        time.sleep(0.1)
        print("Ready to detect colors...\n")

        self.victim_counter = 0.0
        self.in_victim = False
        self.victim_detected = False
        #self.running = True

    def get_and_update_colour(self):
        if True:
            
              print("Colours are...\n")
              data = self.bus.read_i2c_block_data(0x44, 0x09, 6) #Selects the right registers
              green = data[1] + data[0]/256 # Calculates the levels of each color [0, 255]
              red = data[3] + data[2]/256
              blue = data[5] + data[4]/256

            # Determines the color based on which has the higher value
              color = ""
              if green > red and green > blue: 
                 color = "Green"
              elif blue > red:
                 color = "Blue"
              else:
                 color = "Red"

              print("RGB(%d %d %d)" % (red, green, blue))

              print("The color is: " + color)

              if color == "Red":
                print("red light on")
                self.led.on()
                sleep(0.1)
                self.victim_detected = True
            
              else:
                  self.led.off()
                  sleep(0.1)
                  self.victim_detected = False

              sleep(0.5)
  
              if self.victim_detected and not self.in_victim:
                self.victim_counter = self.victim_counter + 1
                self.in_victim = True
              elif not self.victim_detected:
                self.in_victim = False
            
           # except Exception as e:
            #    print(f"Error in color detection thread: {e}")
             #   break

    def get_victim_counter(self):
        return self.victim_counter
    
    def stop(self):
        self.running = False

    #def _safe_sleep(self, total_seconds):
     #   """Sleep but check if stopped."""
      #  sleep_interval = 0.1  # small chunks
       # slept = 0.0
        #while self.running and slept < total_seconds:
         #   time.sleep(sleep_interval)
          #  slept += sleep_interval 


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

        self.stop_distance = 0.4
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.15
        self.tele_twist.angular.z = 0.0
        self.tele_twist.linear.y = 0.0

        self.base_linear_speed = 0.2
        self.base_angular_speed = 1.3

        self.narrow_passage_linear_speed = 0.15
        self.narrow_passage_angular_speed = 0.8

        self.average_linear_speed = 0.0
        self.speed_updates = 0.0
        self.speed_accumulation = 0.0

        self.collision_counter = 0.0
        self.in_collision = False
 
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

        self.timer = self.create_timer(0.08, self.timer_callback)

    #filtering data
    def filter_scan_ranges(self, scan_ranges):
        if not scan_ranges:
            return []
        
        MIN_RANGE = 0.15  #minimum valid distance in meters
        MAX_RANGE = 3.5   #maximum valid distance in meters
        
        filtered_ranges = []
        
        for range_value in scan_ranges:
            #check if value is NaN or inf
            if not isinstance(range_value, (int, float)) or range_value != range_value:  #NaN check
                filtered_ranges.append(MAX_RANGE)
            #check if value is below minimum threshold
            elif range_value < MIN_RANGE:
                filtered_ranges.append(MIN_RANGE)
            #check if value is above maximum threshold
            elif range_value > MAX_RANGE:
                filtered_ranges.append(MAX_RANGE)
            #value is within valid range
            else:
                filtered_ranges.append(range_value)
        
        return filtered_ranges
        

    def get_average_speed(self):
        return self.speed_accumulation/self.speed_updates

    def get_collision_count(self):
        return self.collision_counter

    def scan_callback(self, msg):
        #self.scan_ranges = msg.ranges
        self.scan_ranges = self.filter_scan_ranges(msg.ranges)
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

    '''def turn_for_duration(self, linear_speed, angular_speed, duration):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.005)  # faster updates

       # self.stop_robot()  # Stop after turning'''

    ############################################################################################################
    ############################################### ROBOT NAVIGATION ###########################################
    ############################################################################################################
    def detect_obstacle(self):
        
        #Finds distance from left, right, front, front-left, front-right to an object 
        ##The self_scan_ranges function returns an array of values. Using min function, the minimum value is taken from the returned data array
       
        obstacle_distance_left = min(self.scan_ranges[-90:-72])
        obstacle_distance_front_left = min(self.scan_ranges[-72:-36])

        #front interval
        #-36 to -1 range used as the robot has a hard time with transition from positive-negative angle range
        obstacle_distance_front_1 = min(self.scan_ranges[-36:-1])
        obstacle_distance_front_2 = min(self.scan_ranges[0:36])
        obstacle_distance_front = min(obstacle_distance_front_1, obstacle_distance_front_2)

        obstacle_distance_front_right = min(self.scan_ranges[36:72])
        obstacle_distance_right = min(self.scan_ranges[72:90])

        #Combining the two right cones
        wide_right = obstacle_distance_right + obstacle_distance_front_right
        #Combining the two left cones
        wide_left = obstacle_distance_left + obstacle_distance_front_left

        #determining passage width based on distance to the passage from left and right cones
        passage_width = obstacle_distance_left + obstacle_distance_right

        #determining if the robot centered in the passage by taking the difference 
        off_center = obstacle_distance_left - obstacle_distance_right

        #taking the absolute value of off_center value
        absolute_off_center = abs(off_center)
       
        twist = Twist()
        
         
        #robot navigation in narrow passage
        if (passage_width < 0.5 and obstacle_distance_front > 0.3):
             print('obstacle in narrow passage')

             #robot slowing down because it is in narrow passage
             if absolute_off_center > 0.05:
                 twist.linear.x = self.narrow_passage_linear_speed
                 twist.angular.z = self.narrow_passage_angular_speed
                
                #if robot is too close to the left of narrow passage, it rotates to the right to realign
                 if off_center > 0:
                     twist.linear.x = 0.05
                     twist.angular.z = self.narrow_passage_angular_speed*0.5
                     #self.turn_for_duration(0.05, 0.5, 0.8)
                
                #otherwise robot is too close to the right of narrow passage, so it rotates to the left to realign
                 else:
                     twist.linear.x = 0.0
                     twist.angular.z = -self.narrow_passage_angular_speed*0.5
            
            #out of narrow passage so returns to base speed
             else:
                 twist.linear.x = self.base_linear_speed
                 twist.angular.z = 0.0

       #detects obstacle in front
        if obstacle_distance_front < 0.25:
             print('obstacle in front')
            
            #if robot is too close obstacle in front, it reverses
             if(self.in_collision == True):
                reverse_twist = Twist()
                reverse_twist.linear.x = -0.1  
                reverse_twist.angular.z = 0.0
                self.cmd_vel_pub.publish(reverse_twist)
                time.sleep(0.4)

             #determining which direction should be taken to escape front obstacle
             #turn to right it there's a obstacle closer to the right
             if wide_right - wide_left > 0.06:
                    print('front: right')
                    #some linear speed for smoother turn
                    twist.linear.x = 0.05
                    twist.angular.z = self.base_angular_speed*0.5 #0.7

            #otherwise turn to left 
             else:
                    print('front: left')
                    twist.linear.x = 0.05
                    twist.angular.z = -self.base_angular_speed*0.5 #0.7

        #obstacle near front left cone
        elif obstacle_distance_front_left < self.stop_distance:
            print('obstacle in front left')
            #sharp turn
            twist.angular.z = self.base_angular_speed*0.7
            twist.linear.x = self.base_linear_speed*0.0 #0.4 #0.6
            #self.turn_for_duration(self.base_linear_speed*0.0 , self.base_angular_speed*0.7, 0.8)
        
        #obstacle near (back) left cone
        elif  obstacle_distance_left < self.stop_distance:
            print('obstacle in left')
            #soft turn
            twist.angular.z = self.base_angular_speed*0.2
            twist.linear.x = self.base_linear_speed*0.7 #0.9
            #self.turn_for_duration(self.base_linear_speed*0.7, self.base_angular_speed*0.2, 0.8)
        
        #obstacle near front right cone
        elif obstacle_distance_front_right < self.stop_distance:
            print('obstacle in front right')
            #sharp turn
            twist.angular.z = -self.base_angular_speed*0.7
            twist.linear.x = self.base_linear_speed*0.4 
            #self.turn_for_duration(-self.base_angular_speed*0.7, self.base_linear_speed*0.4, 0.8)
        
        #obstacle near (back) right cone
        elif  obstacle_distance_right < self.stop_distance:
            print('obstacle in right')
            #soft turn
            twist.angular.z = -self.base_angular_speed*0.2
            twist.linear.x = self.base_linear_speed*0.0 
            #self.turn_for_duration(-self.base_angular_speed*0.2, self.base_linear_speed*0.0,  0.8)

        else:
         twist = self.tele_twist

         #publish action
        self.cmd_vel_pub.publish(twist)

        #calculations for average speed 
        self.speed_updates = self.speed_updates + 1
        self.speed_accumulation = self.speed_accumulation + twist.linear.x

       #############################################################################################################
       ########################################## COLLISION DETECTION ##############################################
       #############################################################################################################

        collision_detected = (obstacle_distance_front < 0.20 or obstacle_distance_left < 0.15 or obstacle_distance_front_left < 0.15 or obstacle_distance_right < 0.15 or obstacle_distance_front_right < 0.15)
        if collision_detected and not self.in_collision:
            self.collision_counter = self.collision_counter + 1
            self.in_collision = True
        elif not collision_detected:
            self.in_collision = False

           
def main(args=None):
    rclpy.init(args=args)
    rgb_victim_detection = RGBVictimDetection()
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    

    #determines how long the program will run
    start_time = time.time() 
    end_time = start_time + 90.0

    #the program will run within the specified time
    while time.time() < end_time:
        #calling obstacle detection and victim detection functions
        rclpy.spin_once(turtlebot3_obstacle_detection, timeout_sec=0.1)
        rgb_victim_detection.get_and_update_colour()


    #stops the motors after time runs out
    turtlebot3_obstacle_detection.stop_robot()

    #prints speed, collision count and victim count
    print("Average speed:", turtlebot3_obstacle_detection.get_average_speed())
    print("\nCollision count:", turtlebot3_obstacle_detection.get_collision_count())
    print("\nVictim count:", rgb_victim_detection.get_victim_counter())


    turtlebot3_obstacle_detection.destroy_node()
    
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
