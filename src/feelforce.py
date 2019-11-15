#!/usr/bin/env python
 
import rospy
import random
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

prev_dist = 0
state = 0
i = 0  

def callBack_dist(msg):
    global prev_dist, state, i
    new_dist = msg.data
    arr = np.arange(-15,15) # Generate an array ranging from -15 to 14
    if new_dist == prev_dist: # Check if the distance has changed since last time
        state = arr[i % len(arr)] # State will take value from -15 to 14 as the callback is called
                                  # this will allow us to rotate the robot for in one direction long enough until it sees a free front
        i += 1
    prev_dist = new_dist
 
def callBack_scan(msg):
    pos = Twist()
    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     
    size = len(msg.ranges) # get the length of scan data
    norm = np.linalg.norm(np.multiply(msg.ranges, np.arange(-size/2,size/2))) # calculate the norm of the array for the next line
    rot = sum(np.multiply(msg.ranges, np.arange(-size/2,size/2))/norm)  # I will deconstruct this section:
                                                                        # np.multiply(msg.ranges, np.arange(-size/2,size/2), multiple scan ranges with an array that ranges from -320 to 320
                                                                        # this will be useful to get positive and negative rotation direction
                                                                        # also notice that the weights at the edges are way higher than the center scan this is useful to make the robot turn faster or slower based on the location of free space
                                                                        # normalize the data just nicer to have data ranging from 0 to 1
                                                                        # sum these values which should make rotation a function of the scam
    front = min(msg.ranges[size/2-50:size/2+50]) # get the smallest distance from the front of the robot which is a range from the middle +-50
    # print(sum(msg.ranges))
    if front < 0.6: # Check if the robot has a wall infront of it
        pos.linear.x = 0
        pos.angular.z = state*0.5 # Rotate based on the state above to avoid wall collision
        pub.publish(pos)

    elif sum(msg.ranges) > 1450: # This statement exists to solve the looping problem in the middle section
                                 # It sums all the laser scan array and this statement turns true when the robot has free space all around 
        pos.linear.x = 0.2*front # moving forward is a function of how far a wall is from the front of the robot
        pos.angular.z = state*0.5 # Rotate based on the state just so that it randomly goes away from this loop problem that we identified
        pub.publish(pos)

    elif front >= 0.7*max(msg.ranges): # Check if the front is the most spacious compared to all the other scans otherwise it will rotate till the front is the 0.7*largest
        pos.linear.x = front # moving forward is a function of how far a wall is from the front of the robot
        pos.angular.z = 0
        pub.publish(pos)
    
    else:
        pos.linear.x = front # moving forward is a function of how far a wall is from the front of the robot
        pos.angular.z = rot # positive rot causes the robot to turn towards the region when ranges are higher meaning that there is space in that direction
        pub.publish(pos)



def main():
    rospy.init_node('ForceMapper')
    rospy.Subscriber("/scan", LaserScan, callBack_scan) # Subscribe to the /scan topic to get laser scan data
    rospy.Subscriber("/dist", Float64, callBack_dist) # Subscribe to the /dist topic to distance travelled data
                                                      # Notice 2 different callbacks are required when a single node is subscribing to 2 topics
    # Spin until ctrl + c
    rospy.spin()
 
if __name__ == '__main__':
    main()
