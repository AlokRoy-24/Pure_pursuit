from cmath import sqrt
from tabnanny import filename_only
from typing import final
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np
import math


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.i = 0
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry,'odom',self.cont_sig,10)

    def cont_sig(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.current_vel = 0.5
        sig = self.pure_pursuit()
        self.publisher_.publish(sig)
    
    def pure_pursuit(self):
        final_signal = Twist()
        cx = np.arange(0.5, 50, 0.5)
        cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
        look_ahead_dist = math.sqrt((pow(cy[self.i]-self.y_pos,2)+pow(cx[self.i]-self.x_pos,2)))
        radius = pow(look_ahead_dist,2)/2*cx[self.i]
        self.i+=1
        omega = self.current_vel/radius
        final_signal.angular.z = float(omega)
        final_signal.linear.x = float(self.current_vel)
        return final_signal
 
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()