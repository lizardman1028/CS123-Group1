from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import numpy as np


IMAGE_WIDTH = 1400


# TODO: Add your new constants here


TIMEOUT = 1.0 #threshold in timer_callback
SEARCH_YAW_VEL = 2.0 #searching constant
TRACK_FORWARD_VEL = 1.0 #tracking constant
KP = -4.0 #proportional gain for tracking


class State(Enum):
   SEARCH = 0
   TRACK = 1


class StateMachineNode(Node):
   def __init__(self):
       super().__init__('state_machine_node')


       self.detection_subscription = self.create_subscription(
           Detection2DArray,
           '/detections',
           self.detection_callback,
           10
       )


       self.command_publisher = self.create_publisher(
           Twist,
           'cmd_vel',
           10
       )


       self.timer = self.create_timer(0.1, self.timer_callback)
       self.state = State.TRACK


       # TODO: Add your new member variables here
       self.x_center = 0.0
       self.last_detection_sec = 0
       self.time_sec = 0


   def detection_callback(self, msg):
       """
       Determine which of the HAILO detections is the most central detected object
       """
  
       x_closest_0 = -2
       for detection in msg.detections:
           x = (2 * detection.bbox.center.position.x / IMAGE_WIDTH) - 1
           if x**2 < x_closest_0**2:
               x_closest_0 = x
       if (x_closest_0 > -2):
           self.x_center = x_closest_0
           self.last_detection_sec = msg.header.stamp.sec


       self.time_sec = msg.header.stamp.sec


   def timer_callback(self):
       """
       Implement a timer callback that sets the moves through the state machine based on if the time since the last detection is above a threshold TIMEOUT
       """


       # SEARCH if > TIMEOUT seconds since detection
       if self.time_sec - self.last_detection_sec > TIMEOUT:
           self.state = State.SEARCH
       else:
           self.state = State.TRACK


       yaw_command = 0.0
       forward_vel_command = 0.0


       if self.state == State.SEARCH:
           yaw_command = SEARCH_YAW_VEL
       elif self.state == State.TRACK:
           yaw_command = KP * self.x_center
           forward_vel_command = TRACK_FORWARD_VEL


       cmd = Twist()
       cmd.angular.z = yaw_command
       cmd.linear.x = forward_vel_command
       self.command_publisher.publish(cmd)


def main():
   rclpy.init()
   state_machine_node = StateMachineNode()


   try:
       rclpy.spin(state_machine_node)
   except KeyboardInterrupt:
       print("Program terminated by user")
   finally:
       zero_cmd = Twist()
       state_machine_node.command_publisher.publish(zero_cmd)


       state_machine_node.destroy_node()
       rclpy.shutdown()


if __name__ == '__main__':
   main()



