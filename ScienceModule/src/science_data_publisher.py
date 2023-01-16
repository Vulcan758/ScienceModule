#!/usr/bin/env python3
import json
import rospy
import serial
from std_msgs.msg import String
from random import uniform
from time import sleep

rospy.init_node("science_data_publisher")

#arduino serial.Serial()

data = {}

pub = rospy.Publisher("/science_data", String, queue_size=10)
rate = rospy.Rate(0.5)


while not rospy.is_shutdown():
    red = uniform(68.9, 70)
    blue = uniform(68.9, 70)
    green = uniform(68.9, 70)    
    
    ax = uniform(68.9, 70)
    ay = uniform(68.9, 70)
    az = uniform(68.9, 70)
  
    data["color"] = {"R": red, "G": green, "B": blue}
    data["IMU"] = {"ax": ax, "ay": ay, "az": az}

    data_string = json.dumps(data)
    pub.publish(data_string)
    rate.sleep()
