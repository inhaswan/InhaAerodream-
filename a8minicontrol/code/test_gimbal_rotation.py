"""
@file test_gimbal_rotation.py
@Description: This is a test script for using the SIYI SDK Python implementation to set/get gimbal rotation
"""

from time import sleep
import sys
import os
  
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
  
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK

def test():
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
    if not cam.connect():
        print("No connection ")
        exit(1)
    print("Current motion mode: ", cam._motionMode_msg.mode)
    
    # cam.requestCenterGimbal()
    # sleep(2)
    while True:
        cam.setPitchAngle(0)
        cam.setYawAngle(-90)
        sleep(4)
        cam.setYawAngle(90)
        cam.setPitchAngle(-90)
        sleep(4)
        


 
    print("Attitude (yaw,pitch,roll) eg:", cam.getAttitude())

    cam.disconnect()

if __name__ == "__main__":
    test()
