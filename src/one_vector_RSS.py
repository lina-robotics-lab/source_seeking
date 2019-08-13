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
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from tf.transformations import euler_from_quaternion

"""Global Constants"""
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

"""Node Class"""
class one_vector_RSS():

    def __init__(self):

        # Publisher and Subscribers
        self.move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10);
        self.pose_subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.light_sub = rospy.Subscriber('sensor_readings', Float32MultiArray, self.light_callback)

#        self.localization = rospy.ServiceProxy('global_localization', Empty, intialization)

        self.pose = Pose()
        self.readings = deque([])
        self.max = 0.0
        self.light = 0.0  #should just be the average
        self.loc = np.array([0, 0])


        # History of readings and states
        # Declare list, pop and add numpy arrays that are states
        self.states = deque([])

        # Track velocity, acceleration
        self.vel = np.array([0.02, 0])
        self.acc = np.array([0, 0])

    # Callbacks to get from subscribers
    def pose_callback(self, data):
        self.pose = data.pose.pose
        self.loc = [self.pose.position.x, self.pose.position.y]

    def light_callback(self, data):
        self.max = 0
        temp = data.data
        for i in temp:
            if i > self.max:
                self.max = i
        print("max: " + str(self.max))
#            print(temp)
        if len(self.readings) >= 80:
            for i in range(8):
                self.readings.popleft()
        self.readings.extend(temp)

    def get_light(self):
        r = rospy.Rate(1)
        while not rospy.core.is_shutdown():
            avg = 0
            for i in range(len(self.readings)):
                avg += self.readings[i]
            avg = avg/len(self.readings)
            print(avg)
            r.sleep()
#            self.light = avg

    def initialization(self):
        rospy.wait_for_service('/global_localization')
        try:
            initial = rospy.ServiceProxy('/global_localization', Empty)
            print("tried")
            return
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return

    # Calculate descent direction
    def calc_descent(self):
        time.sleep(5)

        self.initialization()

        twist = Twist()
        twist.linear.x = self.vel[0]
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.vel[1]

        print(twist)
        self.move_pub.publish(twist)

#        # Transmit the current velocity, then change it based on accel
        r = rospy.Rate(5)
        while not rospy.core.is_shutdown():
            # Add next state
            self.get_light()
            temp = np.append(self.loc,self.light)
#            print(temp)
            if len(self.states) >= 20:
                self.states.popleft()
            self.states.append(temp)


            # Calculate descent direction
            sum = np.array([0.0, 0.0])
            weight = 0.0
            gamma = -6
            factor = 0.1
            curState = self.states[-1]
#            print("weight")
            for i in range(0,len(self.states)-1):
                dif = self.states[i][0:2] - curState[0:2]
                dif_norm = np.linalg.norm(dif)
                weight = (1/np.power(2, dif_norm)) - np.power(10,(gamma*dif_norm))
                light_dif = weight * (self.states[i][2] - self.light) /(dif_norm)
                vec = light_dif * dif
                sum += vec
            sum = sum/np.linalg.norm(sum)
            if(np.any(np.isnan(sum))):
                continue
            var_mag = 0.3
            sum += [random.gauss(0,1) * var_mag, random.gauss(0,1)* var_mag]
            sum = np.array(factor * sum/np.linalg.norm(sum))
#            print("sum")
#            print(sum)

            self.acc = np.matmul(np.array([[1.0, 0],[0,0.3]]),np.transpose(np.array(self.transmit(sum))))
#            print(self.acc)
            self.vel = self.vel + self.acc*0.2
            if(np.abs(self.vel[0]) > 0.08):
                print('check')
                self.vel[0] = np.sign(self.vel[0])*0.08
            if(np.abs(self.vel[1]) > 1.00):
                self.vel[1] = np.sign(self.vel[1])*1.00

            twist = Twist()
            twist.linear.x = self.vel[0]
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.vel[1]

            print(twist)
            self.move_pub.publish(twist)

            r.sleep()

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.move_pub.publish(twist)

        return


    # Function to process vector into motion commands

    # Function to transmit the information - need transforms
    def transmit(self, sum):
        qw = self.pose.orientation.w
        qx = self.pose.orientation.x
        qy = self.pose.orientation.y
        qz = self.pose.orientation.z
        (roll, pitch, yaw) = euler_from_quaternion((qx, qy, qz, qw))

        bot_vel = np.array([[np.cos(yaw), -np.sin(yaw)],[np.sin(yaw), np.cos(yaw)]])
        bot_control = np.matmul(bot_vel, np.transpose(sum))
        print(bot_control)
        acc = np.dot(bot_control, [1,0])
        ang_acc = np.dot(bot_control, [0, 1])

        return [acc, ang_acc]


"""Intialize and run main function"""
if __name__=='__main__':
    rospy.init_node('one_vector_RSS', anonymous=False)
    RSS = one_vector_RSS()

    if not rospy.core.is_initialized():
        raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
    logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())

    try:
        time.sleep(1)
#        RSS.initialization()
        RSS.get_light()
#        RSS.calc_descent()
    except KeyboardInterrupt:
        logdebug("keyboard interrupt, shutting down")
        rospy.core.signal_shutdown('keyboard interrupt')
