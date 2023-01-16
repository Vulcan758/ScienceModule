#!/usr/bin/env python3
import json
import rospy
import serial
from std_msgs.msg import String
from random import uniform
from time import sleep
import csv

class SciDataReceive():
    def __init__(self):
        rospy.init_node("science_data_saver")
        rospy.Subscriber("/science_data", String, self.science_callback)
        self.data = None
        self.rate = rospy.Rate(2)

    def science_callback(self, msg):
        self.data = [json.loads(msg.data)]
    
    def update_csv(self, data):

        headers = data[0].keys()
        print(headers)
        print(type(data))
        with open("test.csv", "a") as f:
            saver = csv.DictWriter(f, fieldnames=headers)
            saver.writeheader()
            print(data)
            saver.writerows(data)

    def main(self):
        print(self.data)
        self.update_csv(self.data)
        self.rate.sleep()


node = SciDataReceive()
if __name__ == "__main__":
    print("Initializing")
    sleep(2)
    print("Started!")
    while not rospy.is_shutdown():
        node.main()