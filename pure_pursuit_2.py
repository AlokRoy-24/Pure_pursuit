from cmath import sqrt
from tabnanny import filename_only
from typing import final
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np
import math
import pandas as pd

kp = 1.0

class MinimalSubscriber(Node):

    def _init_(self):
        super()._init_('minimal_subscriber')
        self.i = 0
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry,'odom',self.cont_sig,10)

    def cont_sig(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        (self.roll,self.pitch,self.yaw) = self.euler_from_quaternion(orientation) 
        sig = self.pure_pursuit()
        self.publisher_.publish(sig)
    
    def pure_pursuit(self):
        df = pd.read_csv('file_name.csv')    #predifined path in file_name.csv
        data = df.to_numpy()
        final_signal = Twist()
        alpha = math.atan2(self.y_pos-data[self.i][1],self.x_pos-data[self.i][0]) # gives the angle between the points.
        self.i+=1

        steering_angle = alpha - self.yaw        #steering angle.              
        final_signal.angular.z = steering_angle
        return final_signal
    
    def euler_from_quaternion(self,quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw                 # yaw gives the orientation of bot along z axis.      

 
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if _name_ == '_main_':
    main()