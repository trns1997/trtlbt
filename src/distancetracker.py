#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy
import time
import math

# Used for localization
import tf 
from tf import transformations

class DistanceTracker(): 
    def __init__(self):
        self.transformationListener = tf.TransformListener()
        self.transformationListener.waitForTransform("/odom", "/base_link", rospy.Time(), rospy.Duration(4.0))
        self.r = rospy.Rate(250) # 250hz
        self.startTime = time.time()
        self.maxDistance = 0
        self.maxPos = [0, 0]
        (self.startTrans, self.startRot) = self.getTranslation()
        self.start()
       
    def getTranslation(self):
        (currentTranslation, currentRotation) = self.transformationListener.lookupTransform("/odom", "/base_link", rospy.Time(0))
        return (currentTranslation, currentRotation)

    def start(self):
        print(self.startTrans, self.startRot)
        while not rospy.is_shutdown():
            try:
                curTime = time.time()
                elapsed = curTime - self.startTime

                (curTrans, curRot) = self.getTranslation()
                dx = curTrans[0] - self.startTrans[0]
                dy = curTrans[1] - self.startTrans[1]
                distanceTraveled = math.sqrt(dx * dx + dy * dy)
                

                if (distanceTraveled > self.maxDistance):
                    self.maxDistance = distanceTraveled
                    self.maxPos = [curTrans[0], curTrans[1]]
                
                rospy.loginfo("Time: %s, Distance traveled: %s, (%s, %s)", str(elapsed), str(self.maxDistance), str(self.maxPos[0]), str(self.maxPos[1]))

            except(tf.LookupException, tf.ConnectivityException):
                continue
            rospy.sleep(0.1)

def main():
    rospy.init_node('DistanceTracker')
    try:
        DistanceTracker()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()