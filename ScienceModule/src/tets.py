#!/usr/bin/env python3
import json
import rospy
import serial
from std_msgs.msg import String
from random import uniform
from time import sleep
import csv

data = ""
state = False

def science_callback(msg):
    global data
    data = msg.data
    update_csv(data)

def update_callback(msg):
    global state
    val = msg.data
    if val == "yes":
        state = True
    
def update_csv(data):
    while data == None:
        pass
    headers = data.keys()
    print(headers)
    with open("test.csv", "a") as f:
        saver = csv.DictWriter(f, fieldnames=headers)
        saver.writeheader()
        print(data)
        saver.writerows(data)

def main(self):
    print(self.data)
    self.update_csv(self.data)
    self.rate.sleep()

rospy.init_node("science_data_saver")
rospy.Subscriber("/science_data", String, science_callback)
rospy.Subscriber("/update_csv_state", String, update_callback)
rate = rospy.Rate(2)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        print(data)
        #update_csv(data)
        rate.sleep()