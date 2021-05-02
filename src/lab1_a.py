#!/usr/bin/env python
import rospy
import numpy as np 

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


def callback(msg):
    print("geldi")

rospy.init_node('open_f')
sub = rospy.Subscriber('/closestPoint',Float64, callback )
rospy.spin()