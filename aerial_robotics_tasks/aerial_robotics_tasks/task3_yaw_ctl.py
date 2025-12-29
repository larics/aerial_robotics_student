#!/usr/bin/env python3

__author__ = 'thaus, amilas'

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import SetParametersResult
from aerial_robotics_tasks.task1_pid import PID
from geometry_msgs.msg import Vector3, PoseStamped, Twist
from std_msgs.msg import Float32
import numpy as np
from aerial_robotics_msgs.msg import PIDController
from crazyflie_interfaces.msg import LogDataGeneric
import math
from tf_transformations import euler_from_quaternion

class YawControl(Node):
    '''
    Class implements ROS2 node for cascade (z, vz) PID control for crazyflie yaw.
    Subscribes to:
        pose       - used to extract z-position of the vehicle
        velocity   - used to extract velocity of the vehicle
        pos_ref    - used to set the reference for z-position
        vel_ref    - used to set the reference for vz-position (useful for testing velocity controller)

    Publishes:
        cmd_vel_legacy  - referent value for thrust
        pid_yaw            - publishes PID-yaw data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_yaw_rate       - publishes PID-yaw_rate data - reference value, measured value, P, I, D and total component (useful for tuning params)
    
    Parameters can be set via ROS2 parameter API for online tuning.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''
        super().__init__('crazyflie_yaw_controller')

        self.start_flag = False         # indicates if we received the first measurement

        self.yaw_sp = 0.0                 # yaw set point
        self.yaw_mv = 0                   # yaw measured value

        self.pid_yaw = PID()              # yaw controller
        
        #########################################################
        #########################################################
        # ADD YOUR CODE HERE
        #########################################################
        #########################################################

        # Yaw PID parameters
        self.declare_parameter('yaw_kp', x)
        self.declare_parameter('yaw_ki', x)
        self.declare_parameter('yaw_kd', x)

        #########################################################
        #########################################################

        # Initialize PID controllers with parameters
        self.pid_yaw.set_kp(self.get_parameter('yaw_kp').value)
        self.pid_yaw.set_ki(self.get_parameter('yaw_ki').value)
        self.pid_yaw.set_kd(self.get_parameter('yaw_kd').value)

        # Add a callback to update parameters dynamically
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create subscriptions
        self.create_subscription(PoseStamped, '/cf_1/pose', self.pose_cb, 1)
        self.create_subscription(LogDataGeneric, '/cf_1/velocity', self.vel_cb, 1)
        self.create_subscription(Float32, 'yaw_ref', self.yaw_ref_cb, 1)

        # Create publishers
        self.pub_pid_yaw = self.create_publisher(PIDController, 'pid_yaw', 1)
        self.publisher = self.create_publisher(Twist, 'yaw_rate', 1)
        
        # Create timer for control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.t_start = self.get_clock().now()  

    def control_loop(self):
        '''
        Runs ROS2 node - computes PID algorithms for yaw control.
        '''

        if not self.start_flag:
            self.get_logger().info('Waiting for velocity measurements.')
            return
            
        #########################################################
        #########################################################
        # ADD YOUR CODE HERE
        #########################################################
        #########################################################
        # Implement PID control here.
        # Reference for yaw is stored in self.yaw_sp.
        # Measured yaw is stored in self.yaw_mv.
        # (180 / math.pi) to convert from rad/s to deg/s as crazyflie uses deg/s for yaw rate command
        # Store the final value in yaw_rate_command



        ########################################################
        ########################################################

        setpoint = Twist()
        setpoint.linear.x = 0.0
        setpoint.linear.y = 0.0
        setpoint.linear.z = 0.0
        setpoint.angular.x = 0.0
        setpoint.angular.y = 0.0
        setpoint.angular.z = - (180.0 / math.pi) * float(yaw_rate_command)
        self.publisher.publish(setpoint)
    
        # Publish PID data - could be useful for tuning
        self.pub_pid_yaw.publish(self.pid_yaw.create_msg())

    def pose_cb(self, msg):
        q = [msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(q)
        self.yaw_mv = yaw

    def vel_cb(self, msg):
        
        if not self.start_flag:
            self.start_flag = True
        self.vz_mv = msg.values[2]

    def yaw_ref_cb(self, msg):
        '''
        Referent yaw callback. Received value is used as a referent yaw.
        :param msg: Type Float32
        '''
        self.yaw_sp = msg.data

    def parameter_callback(self, params):
        '''
        Callback to update parameters dynamically.
        :param params: list of parameters
        :return: result of setting parameters
        '''
        for param in params:
            if param.name == 'yaw_kp':
                self.pid_yaw.set_kp(param.value)
            elif param.name == 'yaw_ki':
                self.pid_yaw.set_ki(param.value)
            elif param.name == 'yaw_kd':
                self.pid_yaw.set_kd(param.value)
            else:
                pass

        return SetParametersResult(successful=True)
    
def main(args=None):
    '''
    Main entry point for ROS2 node.
    '''
    rclpy.init(args=args)
    yaw_ctl = YawControl()
    rclpy.spin(yaw_ctl)
    yaw_ctl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
