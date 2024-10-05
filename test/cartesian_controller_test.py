#!/usr/bin/env python

from ur3_leader_follower.cartesian_controller import CartesianController
from ur3_leader_follower.msg import Error
import time

if __name__ == "__main__":
    gains = {'l_p': 1, 'l_i': 1, 'l_d': 0,
             'a_p': 0, 'a_i': 0, 'a_d': 0}

    cart_controller = CartesianController(gains, 0.002)

    error = Error()
    error.linear.x = 0.5
    error.linear.y = 1
    error.linear.z = 0
    error.angular.x = 0
    error.angular.y = 0
    error.angular.z = 0

    print(cart_controller.calculate_twist(error))

    time.sleep(0.5)

    print(cart_controller.calculate_twist(error))

    cart_controller.reset()

    print(cart_controller.calculate_twist(error))
    

    
