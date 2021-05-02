#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 10
kd = 0
ki = 0


servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0
cTime = 0.0
pTime = 0.0
dt = 0.0

errorInt = 0.0
errorDer = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.5 # meters
DESIRED_DISTANCE_LEFT = 0.9
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
L = 0.9



class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=10)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        return 0.0

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kdi
        global dt
        global errorInt
        global errorDer
        global drive_msg


        errorInt = errorInt + error*dt
        errorDer = (error - prev_error)/dt
        pidOut = kp*error + ki*errorInt + kd*errorDer
        angle = -1*pidOut
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = angle

        if (angle >= math.radians(0.0) and  angle <= math.radians(10)):
            drive_msg.drive.speed = 1.5
        elif (angle > math.radians(10) and angle <= math.radians(20)):
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5
            
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        return 0.0 

    def lidar_callback(self, data):
        global L
        global error
        global prev_error
        global cTime
        global pTime
        global dt

        
        pTime = cTime
        cTime = data.header.stamp.nsecs

        dt = (cTime - pTime)*1e-9

        rangeClosest = 1e10
        rangeFarthest = 0

        closestIdx = 0
        farthestIdx = 0

        #print(data.ranges)

        for idx, r in enumerate(data.ranges):
            if np.isnan(r) or np.isinf(r):
                continue
            if r < rangeClosest:
                rangeClosest = r
                closestIdx = idx
            if r > rangeFarthest:
                rangeFarthest = r
                farthestIdx = idx


        closestAngle = (float(closestIdx)/1080)*6.28*(180/math.pi)
        farthestAngle = (float(farthestIdx)/1080)*6.28*(180/math.pi)

        # find the distance at 90 degree
        th90_i = int(math.ceil(math.radians(90)*1080/6.28))

        # find the distance at 160 degree
        th160_i = int(math.ceil(math.radians(160)*1080/6.28))

        a = data.ranges[th160_i]
        b = data.ranges[th90_i]

        alpha = math.atan((a*math.cos(math.radians(70)) - b)/(a*math.sin(math.radians(70))))
        Dt = b*math.cos(alpha)

        Dt1 = Dt + L*math.sin(alpha)

        #print (Dt)
        #print(math.degrees(alpha))

        prev_error = error
        error = DESIRED_DISTANCE_RIGHT - Dt1

        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
