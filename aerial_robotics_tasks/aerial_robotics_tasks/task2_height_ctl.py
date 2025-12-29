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

class HeightControl(Node):
    '''
    Class implements ROS2 node for cascade (z, vz) PID control for crazyflie height.
    Subscribes to:
        pose       - used to extract z-position of the vehicle
        velocity   - used to extract velocity of the vehicle
        pos_ref    - used to set the reference for z-position
        vel_ref    - used to set the reference for vz-position (useful for testing velocity controller)

    Publishes:
        cmd_vel_legacy  - referent value for thrust
        pid_z           - publishes PID-z data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vz          - publishes PID-vz data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Parameters can be set via ROS2 parameter API for online tuning.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''
        super().__init__('crazyflie_z_controller')

        self.start_flag = False         # indicates if we received the first measurement

        self.z_sp = 1.0                 # z-position set point
        self.z_mv = 0                   # z-position measured value
        self.pid_z = PID()              # pid instance for z control

        self.vz_sp = 0                  # vz velocity set_point
        self.vz_mv = 0                  # vz velocity measured value
        self.pid_vz = PID()             # pid instance for z-velocity control

        self.first_pass = False         # referent motors velocity, computed by PID cascade

        # Set limits for PID controllers
        self.pid_vz.set_lim_up(1350.0)    # max value for motor velocity
        self.pid_vz.set_lim_low(-1350.0)  # min value for motor velocity
        self.pid_z.set_lim_up(5)          # max vertical ascent speed
        self.pid_z.set_lim_low(-5)        # max vertical descent speed

        #########################################################
        #########################################################
        # ADD YOUR CODE HERE
        #########################################################
        #########################################################
        # Declare ROS2 parameters
        # Gains for z controller
        self.declare_parameter('z_kp', x)
        self.declare_parameter('z_ki', x)
        self.declare_parameter('z_kd', x)
        # Gains for vz controller
        self.declare_parameter('vz_kp', x)
        self.declare_parameter('vz_ki', x)
        self.declare_parameter('vz_kd', x)
        # Motor hovering velocity. This should be used for computing the control value.
        self.declare_parameter('motor_velocity_hover', x)

        #########################################################
        #########################################################

        # Initialize PID controllers with parameters
        self.pid_vz.set_kp(self.get_parameter('vz_kp').value)
        self.pid_vz.set_ki(self.get_parameter('vz_ki').value)
        self.pid_vz.set_kd(self.get_parameter('vz_kd').value)
        
        self.pid_z.set_kp(self.get_parameter('z_kp').value)
        self.pid_z.set_ki(self.get_parameter('z_ki').value)
        self.pid_z.set_kd(self.get_parameter('z_kd').value)

        # Add a callback to update parameters dynamically
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create subscriptions
        self.create_subscription(PoseStamped, '/cf_1/pose', self.pose_cb, 1)
        self.create_subscription(LogDataGeneric, '/cf_1/velocity', self.vel_cb, 1)
        self.create_subscription(Vector3, 'vel_ref', self.vel_ref_cb, 1)
        self.create_subscription(Vector3, 'pos_ref', self.pos_ref_cb, 1)

        # Create publishers
        self.pub_pid_z = self.create_publisher(PIDController, 'pid_z', 1)
        self.pub_pid_vz = self.create_publisher(PIDController, 'pid_vz', 1)
        self.pub_thrust = self.create_publisher(Twist, 'thrust', 1)
        
        # Create timer for control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.t_start = self.get_clock().now()
        
        self.motor_velocity_hover = self.get_parameter('motor_velocity_hover').value

    def control_loop(self):
        '''
        Runs ROS2 node - computes PID algorithms for z and vz control.
        '''

        if not self.start_flag:
            self.get_logger().info('Waiting for velocity measurements.')
            return
            
        #########################################################
        #########################################################
        # ADD YOUR CODE HERE
        #########################################################
        #########################################################
        # Implement cascade PID control here.
        # Reference for z is stored in self.z_sp.
        # Measured z-position is stored in self.z_mv.
        # If you want to test only vz - controller, the corresponding reference is stored in self.vz_sp.
        # Measured vz-velocity is stored in self.vz_mv
        # The control value should be set to motor_velocity variable, which is used to compute the setpoint.



        ########################################################
        ########################################################

        # Publish PID data - could be useful for tuning
        setpoint = Twist()
        setpoint.linear.x = 0.0
        setpoint.linear.y = 0.0
        # Note: formula for converting motor velocity in rad/s to PWM control signal for crazyflies
        setpoint.linear.z = 24.5307*(float(motor_velocity) - 380.8359)
        setpoint.angular.x = 0.0
        setpoint.angular.y = 0.0
        setpoint.angular.z = 0.0
        self.pub_thrust.publish(setpoint)

        self.pub_pid_z.publish(self.pid_z.create_msg())
        self.pub_pid_vz.publish(self.pid_vz.create_msg())


    def pose_cb(self, msg):
        
        self.z_mv = msg.pose.position.z

    def vel_cb(self, msg):
        
        if not self.start_flag:
            self.start_flag = True
        self.vz_mv = msg.values[2]

    def vel_ref_cb(self, msg):
        '''
        Referent velocity callback. Use received velocity values when during initial tuning
        velocity controller (i.e. when position controller still not implemented).
        :param msg: Type Vector3
        '''
        self.vz_sp = msg.z

    def pos_ref_cb(self, msg):
        '''
        Referent position callback. Received value (z-component) is used as a referent height.
        :param msg: Type Vector3
        '''
        self.z_sp = msg.z

    def parameter_callback(self, params):
        '''
        Callback to update parameters dynamically.
        :param params: list of parameters
        :return: result of setting parameters
        '''
        for param in params:
            if param.name == 'z_kp':
                self.pid_z.set_kp(param.value)
            elif param.name == 'z_ki':
                self.pid_z.set_ki(param.value)
            elif param.name == 'z_kd':
                self.pid_z.set_kd(param.value)
            elif param.name == 'vz_kp':
                self.pid_vz.set_kp(param.value)
            elif param.name == 'vz_ki':
                self.pid_vz.set_ki(param.value)
            elif param.name == 'vz_kd':
                self.pid_vz.set_kd(param.value)
            elif param.name == 'motor_velocity_hover':
                self.motor_velocity_hover = param.value
            else:
                pass

        return SetParametersResult(successful=True)
    
def main(args=None):
    '''
    Main entry point for ROS2 node.
    '''
    rclpy.init(args=args)
    height_ctl = HeightControl()
    rclpy.spin(height_ctl)
    height_ctl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
