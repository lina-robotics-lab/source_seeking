#!/usr/bin/env python

import time
import random
from collections import deque

import os
from rospy.core import logdebug

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg

from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist

"""Global Constants"""
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

"""Node Class"""
class one_vector_RSS():

    def __init__(self):

        # Publisher and Subscribers
        self.move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10);
        #Subscribe to a position thing - either amcl or odom  make sure to numpy serialize
        self.light_sub = rospy.Subscriber('sensor_readings', Float32MultiArray, self.light_callback)

        self.pose = Pose()
        self.light = Float32()  #should just be the average
        self.loc = np.array([0, 0])


        # History of readings and states
        # Declare list, pop and add numpy arrays that are states
        self.states = deque([])

        # Track velocity, acceleration
        self.vel = [0, 0]
        self.acc = [0, 0]

    # Callbacks to get from subscribers
    def pose_callback(self, data):
        self.pose = data.pose.pose
        self.loc = [self.pose.x, self.pose.y]

    def light_callback(self, data):
        avg = 0
        for i in range(len(data.data)):
            avg += data.data[i]
        avg = avg/len(data.data)
        self.light = avg


    # Calculate descent direction
    def calc_descent(self):

#        # Calculate descent direction
#        sum = np.array([0, 0])
#        gamma = 3
#        factor = 0.1
#        for i in range(len(self.states)):
#            weight += (1/np.power(np.linalg.norm(self.states[i][1:2] - self.loc), gamma))
#            sum += weight * (self.states[i][3] - self.light) * self.states[i][1:2]
#        sum = sum/np.norm(sum)
#        var_mag = 0.3
#        sum += randn(1,2) * var_mag
#        sum = factor * sum/np.norm(sum)

#        # Transmit the current velocity, then change it based on accel
        r = rospy.Rate(10)
        while not rospy.core.is_shutdown():
            print(self.light)
            r.sleep()

        return


    # Function to process vector into motion commands

    # Function to transmit the information - need transforms
    def transmit(self):
        qw = self.pose.orientation.w
        qx = self.pose.orientation.x
        qy = self.pose.orientation.y
        qz = self.pose.orientation.z
        (roll, pitch, yaw) = euler_from_quaternion((qx, qy, qz, qw))

        bot_vel = np.array([[np.cos(yaw), -np.sin(yaw)],[np.sin(yaw), np.cos(yaw)]]) * self.vel



        return


"""Intialize and run main function"""
if __name__=='__main__':
    rospy.init_node('one_vector_RSS', anonymous=False)
    RSS = one_vector_RSS()

    if not rospy.core.is_initialized():
        raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
    logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())

    try:
        RSS.calc_descent()
    except KeyboardInterrupt:
        logdebug("keyboard interrupt, shutting down")
        rospy.core.signal_shutdown('keyboard interrupt')
