#!/usr/bin/env python3

__author__ = 'amilas'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Mux(Node):
    '''
    Class implements ROS2 node for multiplexing thrust, yaw_rate, roll and pitch commands.
    Subscribes to:
        thrust_topic      - receives thrust command (linear.z)
        yaw_rate_topic    - receives yaw_rate command (angular.z)
        roll_pitch_topic  - receives roll and pitch commands (linear.y, linear.x)

    Publishes:
        /cf_1/cmd_vel_legacy  - combined Twist message with thrust, yaw_rate, roll and pitch
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''
        super().__init__('mux_node')

        self.start_flag = False         # indicates if we received the first measurement
        self.first_pass = True          # flag for first control loop pass
        
        self.thrust_command = 0.0       # received thrust command
        self.yaw_rate_command = 0.0     # received yaw_rate command
        self.roll_command = 0.0         # received roll command
        self.pitch_command = 0.0        # received pitch command

        # Create subscriptions
        self.create_subscription(Twist, 'thrust', self.thrust_cb, 1)
        self.create_subscription(Twist, 'yaw_rate', self.yaw_rate_cb, 1)
        self.create_subscription(Twist, 'roll_pitch', self.roll_pitch_cb, 1)

        # Create publisher for multiplexed command
        self.cmd_vel_pub = self.create_publisher(Twist, '/cf_1/cmd_vel_legacy', 1)
        
        # Create timer for control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    def control_loop(self):
        '''
        Runs ROS2 node - multiplexes thrust, yaw_rate, roll and pitch into single Twist message.
        '''

        if not self.start_flag:
            self.get_logger().info('Waiting for measurements.')
            return
            
        ########################################################
        ########################################################
        # Multiplex thrust, yaw_rate, roll and pitch commands
        
        setpoint = Twist()
        
        if self.first_pass:
            self.get_logger().info('Starting mux control.')
            self.first_pass = False
            # Publish all zeros on first pass (crazyflie constraint)   
            setpoint.linear.x = 0.0
            setpoint.linear.y = 0.0
            setpoint.linear.z = 0.0
            setpoint.angular.x = 0.0
            setpoint.angular.y = 0.0
            setpoint.angular.z = 0.0
        else:
            # Combine thrust, yaw_rate, roll and pitch commands
            setpoint.linear.x = float(self.pitch_command)
            setpoint.linear.y = float(self.roll_command)
            setpoint.linear.z = float(self.thrust_command)
            setpoint.angular.x = 0.0
            setpoint.angular.y = 0.0
            setpoint.angular.z = float(self.yaw_rate_command)
            
        ########################################################
        ########################################################
        
        self.cmd_vel_pub.publish(setpoint)

    def thrust_cb(self, msg):
        '''
        Thrust command callback. Receives thrust value from height controller.
        :param msg: Type Twist (uses linear.z)
        '''
        if not self.start_flag:
            self.start_flag = True
        self.thrust_command = msg.linear.z

    def yaw_rate_cb(self, msg):
        '''
        Yaw rate command callback. Receives yaw_rate value from yaw controller.
        :param msg: Type Twist (uses angular.z)
        '''
        self.yaw_rate_command = msg.angular.z

    def roll_pitch_cb(self, msg):
        '''
        Roll and pitch command callback. Receives roll and pitch values from horizontal controller.
        :param msg: Type Twist (roll = linear.y, pitch = -linear.x)
        '''
        self.roll_command = msg.linear.y
        self.pitch_command = msg.linear.x
          
def main(args=None):
    '''
    Main entry point for ROS2 node.
    '''
    rclpy.init(args=args)
    mux = Mux()
    rclpy.spin(mux)
    mux.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
