from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import numpy as np

IMAGE_WIDTH = 1400

# TODO: Add your new constants here

TIMEOUT = pass #TODO threshold in timer_callback
SEARCH_YAW_VEL = pass #TODO searching constant
TRACK_FORWARD_VEL = pass #TODO tracking constant
KP = pass #TODO proportional gain for tracking

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
        self.kp = pass # TODO

    def detection_callback(self, msg):
        """
        Determine which of the HAILO detections is the most central detected object
        """
        pass # TODO: Part 1

    def timer_callback(self):
        """
        Implement a timer callback that sets the moves through the state machine based on if the time since the last detection is above a threshold TIMEOUT
        """
        
        if False: # TODO: Part 3.2
            self.state = State.SEARCH
        else:
            self.state = State.TRACK

        yaw_command = 0.0
        forward_vel_command = 0.0

        if self.state == State.SEARCH:
            pass # TODO: Part 3.1
        elif self.state == State.TRACK:
            pass # TODO: Part 2 / 3.4

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
