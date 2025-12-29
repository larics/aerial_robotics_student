from datetime import datetime
from aerial_robotics_msgs.msg import PIDController 
import rclpy
from builtin_interfaces.msg import Time

class PID:
    """ 
        This class implements a simple PID control algorithm.
    """
   
    def __init__(self):
        """
            Initializes PID gains (proportional - kp, integral - ki, derivative - kd) and control values to zero.
        """ 
        
        # initialize gains
        self.kp = 0.0     # proportional gain
        self.ki = 0.0     # integral gain
        self.kd = 0.0     # derivative gain

        # initialize control values
        self.up = 0.0                     # P part
        self.ui = 0.0                     # I part
        self.ui_old = 0.0                 # I part from previous step
        self.ud = 0.0                     # D part
        self.u = 0.0                      # total control value
        self.lim_up = float("inf")      # control value upper limit
        self.lim_low = -float("inf")    # control value lower limit

        # init referent control value (set-value)
        self.ref = 0

        # init measure control value
        self.meas = 0

        # init time stamp of the previous algorithm step
        self.t_old = datetime.now()

        # init error from the previous algorithm step
        self.error_old = 0.0

        # flag indicates first step of the algorithm
        self.firstPass = True

        # ros message formed when create_msg method is called
        self.pid_msg = PIDController()
      
    def reset(self):
        ''' Resets pid algorithm by setting all P,I,D parts to zero'''
        self.up = 0.0
        self.ui = 0.0
        self.ui_old = 0.0
        self.ud = 0.0
        self.u = 0.0
        self.t_old = datetime.now()

    def set_kp(self, invar):
        """ Set proportional gain. """
        self.kp = invar

    def get_kp(self):
        """Returns proportional gain"""
        return self.kp

    def set_ki(self, invar):
        """ Set integral gain. """
        self.ki = invar

    def get_ki(self):
        """Returns integral gain"""
        return self.ki

    def set_kd(self, invar):
        """ Set derivative gain. """
        self.kd = invar

    def get_kd(self):
        """Returns derivative gain"""
        return self.kd

    def set_lim_up(self, invar):
        """Set PID upper limit value"""
        self.lim_up = invar

    def get_lim_up(self):
        """Returns PID upper limit value"""
        return self.lim_up

    def set_lim_low(self, invar):
        """Set PID lower limit value"""
        self.lim_low = invar

    def get_lim_low(self):
        """Returns PID lower limit value"""
        return self.lim_low

    def compute(self, ref, meas):
        '''
        Performs a PID computation and returns a control value based on
        the elapsed time (dt) and the error signal.
        :param ref: referent value
        :param meas: measured value
        :return: control value
        '''

        if self.firstPass:
            # This is the first step of the algorithm
            # Init time stamp and error
            self.t_old = datetime.now()
            self.error_old = ref - meas
            self.firstPass = False
        else:
            #########################################################
            #########################################################
            # ADD YOUR CODE HERE
            #########################################################
            #########################################################
            # You should compute elapsed time from the previous step (dt, in seconds),
            # compute proportional part (self.up),
            # compute integral part (self.ui),
            # compute derivative part (self.ud),
            # compute total control value (self.u),
            # implement saturation function and antiwind-up



            # End of added code
            ###############################################
            ###############################################

        return self.u

    def get_pid_values(self):
        """ Returns P, I, D components and total control value
        """
        return [self.up, self.ui, self.ud, self.u]

    def create_msg(self):
        """ Returns ros message of type PIDController
        """
        self.pid_msg.ref = float(self.ref)
        self.pid_msg.meas = float(self.meas)
        self.pid_msg.p = float(self.up)
        self.pid_msg.i = float(self.ui)
        self.pid_msg.d = float(self.ud)
        self.pid_msg.u = float(self.u)
        # Set timestamp using current time
        now = datetime.now()
        self.pid_msg.header.stamp.sec = int(now.timestamp())
        self.pid_msg.header.stamp.nanosec = int((now.timestamp() % 1) * 1e9)
        return self.pid_msg

