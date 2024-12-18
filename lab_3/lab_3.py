import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
np.set_printoptions(precision=3, suppress=True)

Kp = .000005
Kd = 0.1

class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )

        self.pd_timer_period = 1.0 / 200  # 200 Hz
        self.ik_timer_period = 1.0 / 20   # 10 Hz
        self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)

        self.joint_positions = None
        self.joint_velocities = None
        self.target_joint_positions = None

        self.ee_triangle_positions = np.array([
            [0.05, 0.0, -0.12],  # Touchdown
            [-0.05, 0.0, -0.12], # Liftoff
            [0.0, 0.0, -0.06]    # Mid-swing
        ])

        center_to_rf_hip = np.array([0.07500, -0.08350, 0])
        self.ee_triangle_positions = self.ee_triangle_positions + center_to_rf_hip
        self.current_target = 0
        self.t = 0

    def listener_callback(self, msg):
        joints_of_interest = ['leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3']
        self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
        self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])

    def forward_kinematics(self, theta1, theta2, theta3):
       
        def rotation_x(angle):
            # rotation about the x-axis implemented for you
            return np.array([
                [1, 0, 0, 0],
                [0, np.cos(angle), -np.sin(angle), 0],
                [0, np.sin(angle), np.cos(angle), 0],
                [0, 0, 0, 1]
            ])

        def rotation_y(angle):
            return np.array([
                [np.cos(angle), 0, np.sin(angle), 0],
                [0, 1, 0, 0],
                [-np.sin(angle), 0, np.cos(angle), 0],
                [0, 0, 0, 1]
            ])
        
        def rotation_z(angle):
            return np.array([
                [np.cos(angle), -np.sin(angle), 0, 0],
                [np.sin(angle), np.cos(angle), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

        def translation(x, y, z):
            # TODO: Implement the translation matrix
            return np.array([
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1]
            ])
            None

        # The translation values are the arm length values

        # T_0_1 (base_link to leg_front_r_1)
        # translate position to that of motor, then rotate by 90 degrees about the x-axis so z matches axis of motor rotation
        #       The position we desire (given)     Rotate the z-axis across the x-axis by 90 (pi/2)   Match z to whatever angle the motor is at (since z represents what we want to rotate)
        T_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta1)

        # T_1_2 (leg_front_r_1 to leg_front_r_2)
        ## TODO: Implement the transformation matrix from leg_front_r_1 to leg_front_r_2
        T_1_2 = translation(0.0, 0.0, 0.039) @ rotation_y(-1.57080) @ rotation_z(theta2)


        # T_2_3 (leg_front_r_2 to leg_front_r_3)
        ## TODO: Implement the transformation matrix from leg_front_r_2 to leg_front_r_3
        T_2_3 = translation(0.0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(theta3)

        # T_3_ee (leg_front_r_3 to end-effector)
        T_3_ee = translation(0.06231, -0.06216, 0.018) 

        # TODO: Compute the final transformation. T_0_ee is a concatenation of the previous transformation matrices
        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        # TODO: Extract the end-effector position. The end effector position is a 3x3 matrix (not in homogenous coordinates)
        # 0 : 3 encompasses rows 0, 1, 2 b/c exclusive bound, then the 3 is the 3rd column which has our x, y, z
        end_effector_position = T_0_ee[0:3 , 3]

        return end_effector_position


    def inverse_kinematics(self, target_ee, initial_guess=[1000, -1000, 1655489]):
        def cost_function(theta):
            # Compute the cost function and the L1 norm of the error
            # return the cost and the L1 norm of the error
            ################################################################################################
            current_ee = self.forward_kinematics(theta[0], theta[1], theta[2])
            cost_dist = target_ee - current_ee
            ################################################################################################
            return np.sum(cost_dist * cost_dist), np.sum(np.abs(cost_dist))

        def gradient(theta, epsilon=1e-3):
            # Compute the gradient of the cost function using finite differences
            ################################################################################################
            theta1 = theta.copy()
            theta1[0] = theta1[0] + epsilon
            grad1 = (cost_function(theta1)[0] - cost_function(theta)[0])/epsilon

            theta2 = theta.copy()
            theta2[1] = theta2[1] + epsilon
            grad2 = (cost_function(theta2)[0] - cost_function(theta)[0])/epsilon

            theta3 = theta.copy()
            theta3[2] = theta3[2] + epsilon
            grad3 = (cost_function(theta3)[0] - cost_function(theta)[0])/epsilon

            ################################################################################################
            return np.array([grad1, grad2, grad3])

        theta = np.array(initial_guess)
        learning_rate = 10 # TODO: Set the learning rate
        max_iterations = 200 # TODO: Set the maximum number of iterations
        tolerance = 1e-3# TODO: Set the tolerance for the L1 norm of the error

        cost_l = []
        for _ in range(max_iterations):
            grad = gradient(theta)

            # Update the theta (parameters) using the gradient and the learning rate
            ################################################################################################
            # TODO (BONUS): Implement the (quasi-)Newton's method for faster convergence
            theta -= learning_rate * grad
            ################################################################################################

            cost, l1 = cost_function(theta)
            # cost_l.append(cost)
            if l1.mean() < tolerance:
                break

        # print(f'Cost: {cost_l}')

        return theta

    def interpolate_triangle(self, t):
        # Intepolate between the three triangle positions in the self.ee_triangle_positions
        # based on the current time t
        ################################################################################################
        if (t % 3 < 1):
            x = np.interp(t%1, [0, 1], [self.ee_triangle_positions[0][0], self.ee_triangle_positions[1][0]])
            y = np.interp(t%1, [0, 1], [self.ee_triangle_positions[0][1], self.ee_triangle_positions[1][1]])
            z = np.interp(t%1, [0, 1], [self.ee_triangle_positions[0][2], self.ee_triangle_positions[1][2]])
        elif (t % 3 < 2):
            x = np.interp(t%1, [0, 1], [self.ee_triangle_positions[1][0], self.ee_triangle_positions[2][0]])
            y = np.interp(t%1, [0, 1], [self.ee_triangle_positions[1][1], self.ee_triangle_positions[2][1]])
            z = np.interp(t%1, [0, 1], [self.ee_triangle_positions[1][2], self.ee_triangle_positions[2][2]])
        else:
            x = np.interp(t%1, [0, 1], [self.ee_triangle_positions[2][0], self.ee_triangle_positions[0][0]])
            y = np.interp(t%1, [0, 1], [self.ee_triangle_positions[2][1], self.ee_triangle_positions[0][1]])
            z = np.interp(t%1, [0, 1], [self.ee_triangle_positions[2][2], self.ee_triangle_positions[0][2]])        
        ################################################################################################
        return np.array([x, y, z])

    def ik_timer_callback(self):
        if self.joint_positions is not None:
            target_ee = self.interpolate_triangle(self.t)
            self.target_joint_positions = self.inverse_kinematics(target_ee, self.joint_positions)
            current_ee = self.forward_kinematics(*self.joint_positions)

            # update the current time for the triangle interpolation
            ################################################################################################
            self.t += self.ik_timer_period
            ################################################################################################
            
            self.get_logger().info(f'Target EE: {target_ee}, Current EE: {current_ee}, Target Angles: {self.target_joint_positions}, Target Angles to EE: {self.forward_kinematics(*self.target_joint_positions)}, Current Angles: {self.joint_positions}')

    def pd_timer_callback(self):
        if self.target_joint_positions is not None:

            command_msg = Float64MultiArray()
            command_msg.data = self.target_joint_positions.tolist()
            self.command_publisher.publish(command_msg)

def main():
    rclpy.init()
    inverse_kinematics = InverseKinematics()
    
    try:
        rclpy.spin(inverse_kinematics)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        # Send zero torques
        zero_torques = Float64MultiArray()
        zero_torques.data = [0.0, 0.0, 0.0]
        inverse_kinematics.command_publisher.publish(zero_torques)
        
        inverse_kinematics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
