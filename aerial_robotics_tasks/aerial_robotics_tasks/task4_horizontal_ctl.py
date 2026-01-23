#!/usr/bin/env python3

__author__ = 'amilas'

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
from tf_transformations import euler_from_quaternion
import math # For sin and cos

class HorizontalControl(Node):
    '''
    Class implements ROS2 node for cascade (x, vx) and (y, vy) PID control for crazyflie horizontal position.
    Subscribes to:
        pose       - used to extract x, y position of the vehicle
        velocity   - used to extract velocity of the vehicle
        pos_ref    - used to set the reference for x, y position
        vel_ref    - used to set the reference for vx, vy velocity (useful for testing velocity controller)

    Publishes:
        roll_pitch  - referent values for roll and pitch
        pid_x       - publishes PID-x data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vx      - publishes PID-vx data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_y       - publishes PID-y data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vy      - publishes PID-vy data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Parameters can be set via ROS2 parameter API for online tuning.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''
        super().__init__('crazyflie_horizontal_controller')

        self.yaw_mv = 0.0

        self.start_flag = False         # indicates if we received the first measurement

        self.x_sp = 0.0                 # x-position set point
        self.x_mv = 0                   # x-position measured value
        self.pid_x = PID()              # pid instance for x control

        self.vx_sp = 0                  # vx velocity set_point
        self.vx_mv = 0                  # vx velocity measured value
        self.pid_vx = PID()             # pid instance for x-velocity control

        self.y_sp = 0.0                 # y-position set point
        self.y_mv = 0                   # y-position measured value
        self.pid_y = PID()              # pid instance for y control

        self.vy_sp = 0                  # vy velocity set_point
        self.vy_mv = 0                  # vy velocity measured value
        self.pid_vy = PID()             # pid instance for y-velocity control

        # Set limits for PID controllers
        
        self.pid_vx.set_lim_up(60)      # max value for pitch (degrees)
        self.pid_vx.set_lim_low(-60)    # min value for pitch (degrees)

        self.pid_x.set_lim_up(5)        # max horizontal speed
        self.pid_x.set_lim_low(-5)      # max horizontal speed

        self.pid_vy.set_lim_up(60)      # max value for roll (degrees)
        self.pid_vy.set_lim_low(-60)    # min value for roll (degrees)

        self.pid_y.set_lim_up(5)        # max horizontal speed
        self.pid_y.set_lim_low(-5)      # max horizontal speed

        #########################################################
        #########################################################
        # ADD YOUR CODE HERE
        #########################################################
        #########################################################
        # Declare ROS2 parameters
        self.declare_parameter('x_kp', x)
        self.declare_parameter('x_ki', x)
        self.declare_parameter('x_kd', x)

        self.declare_parameter('vx_kp', x)
        self.declare_parameter('vx_ki', x)
        self.declare_parameter('vx_kd', x)

        self.declare_parameter('y_kp', x)
        self.declare_parameter('y_ki', x)
        self.declare_parameter('y_kd', x)

        self.declare_parameter('vy_kp', x)
        self.declare_parameter('vy_ki', x)
        self.declare_parameter('vy_kd', x)

        #########################################################
        #########################################################

        # Initialize PID controllers with parameters
        self.pid_vx.set_kp(self.get_parameter('vx_kp').value)
        self.pid_vx.set_ki(self.get_parameter('vx_ki').value)
        self.pid_vx.set_kd(self.get_parameter('vx_kd').value)
        
        self.pid_x.set_kp(self.get_parameter('x_kp').value)
        self.pid_x.set_ki(self.get_parameter('x_ki').value)
        self.pid_x.set_kd(self.get_parameter('x_kd').value)

        self.pid_vy.set_kp(self.get_parameter('vy_kp').value)
        self.pid_vy.set_ki(self.get_parameter('vy_ki').value)
        self.pid_vy.set_kd(self.get_parameter('vy_kd').value)
        
        self.pid_y.set_kp(self.get_parameter('y_kp').value)
        self.pid_y.set_ki(self.get_parameter('y_ki').value)
        self.pid_y.set_kd(self.get_parameter('y_kd').value)

        # Add a callback to update parameters dynamically
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create subscriptions
        self.create_subscription(PoseStamped, '/cf_1/pose', self.pose_cb, 1)
        self.create_subscription(LogDataGeneric, '/cf_1/velocity', self.vel_cb, 1)
        self.create_subscription(Vector3, 'vel_ref', self.vel_ref_cb, 1)
        self.create_subscription(Vector3, 'pos_ref', self.pos_ref_cb, 1)

        # Create publishers
        self.pub_pid_x = self.create_publisher(PIDController, 'pid_x', 1)
        self.pub_pid_vx = self.create_publisher(PIDController, 'pid_vx', 1)
        self.pub_pid_y = self.create_publisher(PIDController, 'pid_y', 1)
        self.pub_pid_vy = self.create_publisher(PIDController, 'pid_vy', 1)
        self.publisher = self.create_publisher(Twist, 'roll_pitch', 1)
        
        # Create timer for control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.t_start = self.get_clock().now()

    def control_loop(self):
        '''
        Runs ROS2 node - computes PID algorithms for x, vx, y, vy control.
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
        # Reference for x is stored in self.x_sp.
        # Measured x-position is stored in self.x_mv.
        # Reference for y is stored in self.y_sp.
        # Measured y-position is stored in self.y_mv.
        # If you want to test only vx/vy - controller, the corresponding reference is stored in self.vx_sp/self.vy_sp.
        # Measured vx-velocity is stored in self.vx_mv
        # Measured vy-velocity is stored in self.vy_mv
        # Current yaw is stored in self.yaw_mv. Use it so controller can work under any yaw.
        # Store your setpoints to pitch_command and roll_command



        ########################################################
        ########################################################

        # In standard circumstances, x axis would be roll, and y axis would be pitch command.
        # However, due to crazyflies specifics, these values are switched and x command is inverted.
        # Within lab, you can consider that x = roll_command; y = pitch_command, and these lines map it
        # correctly for crazyflies.
        setpoint = Twist()
        setpoint.linear.x = - float(pitch_command)
        setpoint.linear.y = float(roll_command)
        setpoint.linear.z = 0.0
        setpoint.angular.x = 0.0
        setpoint.angular.y = 0.0
        setpoint.angular.z = 0.0
        self.publisher.publish(setpoint)
       
        # Publish PID data - could be useful for tuning
        self.pub_pid_x.publish(self.pid_x.create_msg())
        self.pub_pid_vx.publish(self.pid_vx.create_msg())
        self.pub_pid_y.publish(self.pid_y.create_msg())
        self.pub_pid_vy.publish(self.pid_vy.create_msg())

    def pose_cb(self, msg):
        
        self.x_mv = msg.pose.position.x
        self.y_mv = msg.pose.position.y

        q = [msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(q)
        self.yaw_mv = yaw
        
    def vel_cb(self, msg):
        
        if not self.start_flag:
            self.start_flag = True
        self.vx_mv = msg.values[0]
        self.vy_mv = msg.values[1]

    def vel_ref_cb(self, msg):
        '''
        Referent velocity callback. Use received velocity values when during initial tuning
        velocity controller (i.e. when position controller still not implemented).
        :param msg: Type Vector3
        '''
        self.vx_sp = msg.x
        self.vy_sp = msg.y
        
    def pos_ref_cb(self, msg):
        '''
        Referent position callback. Received value (x, y components) is used as a referent position.
        :param msg: Type Vector3
        '''
        self.x_sp = msg.x
        self.y_sp = msg.y
     
    def parameter_callback(self, params):
        '''
        Callback to update parameters dynamically.
        :param params: list of parameters
        :return: result of setting parameters
        '''
        for param in params:
            if param.name == 'x_kp':
                self.pid_x.set_kp(param.value)
            elif param.name == 'x_ki':
                self.pid_x.set_ki(param.value)
            elif param.name == 'x_kd':
                self.pid_x.set_kd(param.value)
            elif param.name == 'vx_kp':
                self.pid_vx.set_kp(param.value)
            elif param.name == 'vx_ki':
                self.pid_vx.set_ki(param.value)
            elif param.name == 'vx_kd':
                self.pid_vx.set_kd(param.value)
            elif param.name == 'y_kp':
                self.pid_y.set_kp(param.value)
            elif param.name == 'y_ki':
                self.pid_y.set_ki(param.value)
            elif param.name == 'y_kd':
                self.pid_y.set_kd(param.value)
            elif param.name == 'vy_kp':
                self.pid_vy.set_kp(param.value)
            elif param.name == 'vy_ki':
                self.pid_vy.set_ki(param.value)
            elif param.name == 'vy_kd':
                self.pid_vy.set_kd(param.value)
            else:
                pass

        return SetParametersResult(successful=True)
    
def main(args=None):
    '''
    Main entry point for ROS2 node.
    '''
    rclpy.init(args=args)
    horizontal_ctl = HorizontalControl()
    rclpy.spin(horizontal_ctl)
    horizontal_ctl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

