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

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class MotionCommand(Node):

    def __init__(self):
        # Initialize the base class
        super().__init__('motion_command')

        # Create a subscriber to the bump_expectation topic
        self.subscription = self.create_subscription(
            Int32,                           # Message type
            '/bump_expectation',             # Topic
            self.bump_expectation_callback,  # Callback function
            10)                              # QoS profile (history depth)
        
        # Create a publisher to publish the command (Go Straight or Twist) to 
        # the turtle
        self.publisher_ = self.create_publisher(
            Twist,                  # Message type 
            '/GZ_cmd_vel',     # Topic
            10)                     # QoS profile (history depth)
        publisher_timer_period = 1 # seconds
        self.publisher_timer = self.create_timer(
            publisher_timer_period, 
            self.command_callback)  # Create timer for the publishers
        
        # Initialize the wall_flag to 0
        self.wall_flag = 0

        # Linear velocity
        self.linear_velocity_straight = 1.0   # when going straight
        self.linear_velocity_turn = 0.        # when turning

        # Angular velocity
        deg2rad = 3.14 / 180.0
        self.angular_velocity_straight = 0.0        # when going straight
        self.angular_velocity_turn = 50.0 * deg2rad # when turning

    def bump_expectation_callback(self, msg):
        # Listen for the bump_expectation topic. If the turtle is too close to
        # the wall, it will receive a 1. If it is not too close to the wall, it
        # will receive a 0.

        # Previous wall_flag
        previous_wall_flag = self.wall_flag

        # New wall_flag
        self.wall_flag = msg.data

        # Check if the wall_flag has changed
        if previous_wall_flag != self.wall_flag:
            if self.wall_flag == 1:
                self.get_logger().info('Getting too close to the wall')
            else:
                self.get_logger().info('Not too close to the wall anymore')

    def command_callback(self):
        # Initialize the message
        msg = Twist()

        # Customize the message based on the wall_flag
        if self.wall_flag == 1: # Too close to the wall
            # Turn
            msg.linear.x = self.linear_velocity_turn
            msg.angular.z = self.angular_velocity_turn

            # Increase linear velocity to avoid getting stuck (i.e., to avoid
            # going in circles in a zone that is too close to the wall)
            self.linear_velocity_turn += 0.1
        else:
            # Go straight
            msg.linear.x = self.linear_velocity_straight
            msg.angular.z = self.angular_velocity_straight

            # Reset linear velocity for the next time the turtle gets too close
            # to the wall
            self.linear_velocity_turn = 0.0

        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    motion_command = MotionCommand()

    rclpy.spin(motion_command)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motion_command.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
