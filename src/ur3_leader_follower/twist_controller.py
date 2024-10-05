#!/usr/bin/env python

import simple_pid
from PyKDL import Vector, Twist

class TwistController:
    def __init__(self, gains, alpha):
        """Cartesian controller that generates a twist velocity given error between an actual 
        and desired pose. This is done using a series of six simple PID controllers. The output is
        then processed using a smoothing filter.

        Args:
            gains {dictionary}: dictionary which contains the PID gains for the linear and angular
            components of the controller
            alpha {float}: constant that determines the strength of the filter that is used to smooth 
            the output twist
        """

        # Linear Controllers
        self.linear_x_pid = simple_pid.PID(Kp=gains['l_p'], Ki=gains['l_i'], Kd=gains['l_d'], setpoint=0)
        self.linear_x_pid.output_limits = (-0.5, 0.5)
        self.linear_y_pid = simple_pid.PID(Kp=gains['l_p'], Ki=gains['l_i'], Kd=gains['l_d'], setpoint=0)
        self.linear_y_pid.output_limits = (-0.5, 0.5)
        self.linear_z_pid = simple_pid.PID(Kp=gains['l_p'], Ki=gains['l_i'], Kd=gains['l_d'], setpoint=0)
        self.linear_z_pid.output_limits = (-0.5, 0.5)

        # Angular Controllers
        self.angular_x_pid = simple_pid.PID(Kp=gains['a_p'], Ki=gains['a_i'], Kd=gains['a_d'], setpoint=0)
        self.angular_y_pid = simple_pid.PID(Kp=gains['a_p'], Ki=gains['a_i'], Kd=gains['a_d'], setpoint=0)
        self.angular_z_pid = simple_pid.PID(Kp=gains['a_p'], Ki=gains['a_i'], Kd=gains['a_d'], setpoint=0)

        # Filtering Constant
        self.alpha = alpha

        # Controller dictionary for easier access
        self.controllers = {'l_x': self.linear_x_pid, 'l_y': self.linear_y_pid, 'l_z': self.linear_z_pid,
                            'a_x': self.angular_x_pid, 'a_y': self.angular_y_pid, 'a_z': self.angular_z_pid}

        # Controller Output dictionarys
        self.raw_output = {'l_x': 0, 'l_y': 0, 'l_z': 0,
                           'a_x': 0, 'a_y': 0, 'a_z': 0}
        self.filtered_output = {'l_x': 0, 'l_y': 0, 'l_z': 0,
                                'a_x': 0, 'a_y': 0, 'a_z': 0}
        
    def update_gains_alpha(self, gains, alpha):
        self.linear_x_pid = simple_pid.PID(Kp=gains['l_p'], Ki=gains['l_i'], Kd=gains['l_d'], setpoint=0)
        self.linear_y_pid = simple_pid.PID(Kp=gains['l_p'], Ki=gains['l_i'], Kd=gains['l_d'], setpoint=0)
        self.linear_z_pid = simple_pid.PID(Kp=gains['l_p'], Ki=gains['l_i'], Kd=gains['l_d'], setpoint=0)
        self.controllers['l_x'] = self.linear_x_pid
        self.controllers['l_y'] = self.linear_y_pid
        self.controllers['l_z'] = self.linear_z_pid
        self.alpha = alpha
        
    def get_gains(self):
        return (self.controllers['l_x'])

    def reset(self):
        """Resets the error of all six controllers and resets the filtered output to zero"""

        for key in self.controllers.keys():
            self.controllers[key].reset()
            self.filtered_output[key] = 0

    def calculate_twist(self, error):
        """Takes in an error msg and outputs a filtered twist

        Args:
            error {Error}: error message with linear and angular error

        Returns:
            twist {PyKDL.Twist}: calculated twist velocity
        """

        # Convert error into dictionary for easier access
        errors = {'l_x': error.linear.x, 'l_y': error.linear.y, 'l_z': error.linear.z,
                  'a_x': error.angular.x, 'a_y': error.angular.y, 'a_z': error.angular.z}

        # Get raw controller output
        for key in self.controllers.keys():
            self.raw_output[key] = self.controllers[key](errors[key])

        # Filter controller output
        for key in self.controllers.keys():
            self.filtered_output[key] = self.alpha * self.raw_output[key] + ((1 - self.alpha) * self.filtered_output[key])

        twist = Twist(Vector(self.filtered_output['l_x'], self.filtered_output['l_y'], self.filtered_output['l_z']),
                      Vector(self.filtered_output['a_x'], self.filtered_output['a_y'], self.filtered_output['a_z']))

        return twist

        

