#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callBack(msg):
    pos = Twist()
    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    size = len(msg.ranges) # get the length of scan data
    front  = np.min(msg.ranges[size/2-30:size/2+30]) # front: middle of scan range +-30 and get the min
    right_range = msg.ranges[0:size/2 - 29] # left: left most to middle - 30
    left_range = msg.ranges[size/2+31:size] # right: middle + 30 to the right most
    print(front, np.mean(right_range))
    if front < 0.7: #To preven the robot from hitting the wall head on
        pos.linear.x = -0.5 # constant linear speed
        pos.angular.z = -2*np.mean(right_range) + np.mean(left_range) # some angulat speed just to get it out from a stuck state
        pub.publish(pos) # publish to cmd node

    elif np.mean(right_range) > 2.25:
        pos.linear.x = 0.5 # constant linear speed
        pos.angular.z = -0.6*np.mean(right_range)
        pub.publish(pos) # publish to cmd node
    
    else:
        pos.linear.x = 0.5
        pos.angular.z = -np.mean(right_range) + np.mean(left_range) # take an averge of the ranges 
                                                                    # on the left hemisphere and right hemispher of the scan
                                                                    # and take the differenece, causing the robot to go to the freest space on the map
        pub.publish(pos) # publish to cmd node

''' This is the simple turn right algo

    front  = np.mean(msg.ranges[size/2-15:size/2+15])
    right = np.mean(msg.ranges[size/2 - 130 - 15:size/2 - 130 + 15])
    left = np.mean(msg.ranges[size/2 + 130 - 15:size/2 + 130 + 15])
    print(front, right, left)
    if right > 0.85:
        pos.linear.x = 0.4
        pos.angular.z = -right
        pub.publish(pos)
    
    elif front < 0.7:
        pos.linear.x = 0
        pos.angular.z = 0.3
        pub.publish(pos)

    else:
        pos.linear.x = 0.3
        pos.angular.z = left
        pub.publish(pos)
'''


def main():
    rospy.init_node('ForceMapper')
    rospy.Subscriber("/scan", LaserScan, callBack)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()


    #if object close to front (always pass string)
    #array of three F,L,R 
    #If no longer object to right(entrance) turn right to follow wall
    #three parrallel while loops 
    #all in feelforce

    