import json
import rospy
import serial
from std_msgs.msg import String
from random import uniform
from time import sleep

data = {}


while True:
    red = uniform(68.9, 70)
    blue = uniform(68.9, 70)
    green = uniform(68.9, 70)
    
    ax = uniform(68.9, 70)
    ay = uniform(68.9, 70)
    az = uniform(68.9, 70)
  
    data["color"] = {"R": red, "G": green, "B": blue}
    data["IMU"] = {"ax": ax, "ay": ay, "az": az}

    print(f"This is dict {data}")

    jstring = json.dumps(data)
    print(f"This is jstring {jstring}")
    sleep(0.5)