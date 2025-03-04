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

from std_msgs.msg import Int32
from turtlesim.msg import Pose

class WallDetector(Node):

    def __init__(self):
        # Initialize the base class
        super().__init__('wall_detector')

        # Create a subscriber to the pose of the turtle
        self.subscription = self.create_subscription(
            Pose,                       # Message type
            '/turtle1/pose',            # Topic
            self.turtle_pose_callback,  # Callback function
            10)                         # QoS profile (history depth)

        # Create a publisher to publish information about hitting the wall
        # This will be an integer (0 or 1) that indicates whether the turtle
        # is too close to the wall
        self.publisher_ = self.create_publisher(
            Int32, 
            '/bump_expectation',
            10)
        publisher_timer_period = 1 # seconds
        self.publisher_timer = self.create_timer(
            publisher_timer_period, 
            self.warning_callback)
        
        # Initialize the wall_flag to 0
        self.wall_flag = 0

    def turtle_pose_callback(self, msg):
        # The pose msg contains three numbers (the x, y, and theta of the 
        # turtle). This can be verified by running the following command:
        # ros2 interface show turtlesim/msg/Pose
        self.turtle_x = msg.x
        self.turtle_y = msg.y
        # turtle_theta = msg.theta

        # Log the pose of the turtle (debug level only, change to info if 
        # needed)
        self.get_logger().debug('Turtle pose: x=%f, y=%f' % \
                               (self.turtle_x, self.turtle_y))

        # Define the boundaries of the domain
        x_min = 2
        x_max = 9
        y_min = x_min
        y_max = x_max

        # Check if the turtle is too close to the wall
        # The domain boundaries (i.e., the walls) are [0, 11]x[0, 11]. If the
        # turtle approaches too much the walls, we will issue a warning, which
        # corresponds to a 1. Otherwise, we will issue a 0, indicating that the
        # turtle is not too close to the wall.
        # The margin is conservative, as the turtle is considered too close to
        # the wall if it goes outside the domain [x_min, x_max]x[y_min, y_max]
        # with x_min = y_min > 0 and x_max = y_max < 11.
        if self.turtle_x < x_min or self.turtle_x > x_max or \
            self.turtle_y < y_min or self.turtle_y > y_max:
            self.wall_flag = 1

            # Log a message to indicate that the turtle is too close to the wall
            self.get_logger().debug('Turtle is too close to the wall')
        else:
            self.wall_flag = 0

            # Log a message to indicate that the turtle is not too close to the 
            # wall
            self.get_logger().debug('Turtle is not too close to the wall')

    def warning_callback(self):

        # Create a message to publish
        msg = Int32()

        # Customize the message based on the wall_flag
        msg.data = self.wall_flag

        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    wall_detector = WallDetector()

    rclpy.spin(wall_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
