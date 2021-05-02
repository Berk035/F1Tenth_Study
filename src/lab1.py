#!/usr/bin/env python
import rospy
import numpy as np 
import math
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

def callback(msg):
    rangeClosest = 1e10
    rangeFarthest = 0.0

    closestIdx = 0
    farthestIdx = 0

    carSpeed = 2

    for idx, r in enumerate(msg.ranges):
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

    a = msg.ranges[th160_i]
    b = msg.ranges[th90_i]

    alpha = math.atan((a*math.cos(math.radians(70)) - b)/(a*math.sin(math.radians(70))))
    Dt = b*math.cos(alpha)

    print (Dt)
    print(math.degrees(alpha))

    #print(rangeFarthest)

    ack_msg = AckermannDriveStamped()
    ack_msg.header.stamp = rospy.Time.now()
    ack_msg.drive.speed= 0
    #ack_msg.drive.steering_angle = 0.25*math.pi* math.cos(2*math.pi*0.5*time.time())
    #ack_msg.drive.steering_angle_velocity = 0.25*math.pi* math.cos(2*math.pi*0.5*time.time())
    pub_drive.publish(ack_msg)
    pub_Closest.publish(rangeClosest)
    pub_Farthest.publish(rangeFarthest)



rospy.init_node('closed_farthest_point')
sub = rospy.Subscriber('/scan', LaserScan, callback, queue_size=10)
pub_Closest = rospy.Publisher('/closestPoint', Float64)
pub_Farthest = rospy.Publisher('/farthestPoint', Float64)
pub_drive = rospy.Publisher('/drive', AckermannDriveStamped,queue_size=1)
rospy.spin()