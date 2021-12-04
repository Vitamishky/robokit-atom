#!/usr/bin/env python3

import rospy
from core.srv import MotionService, WalkService, ServoService
import time

def walk_client(n, step, side, ang):
    rospy.wait_for_service('walk_service')
    try:
        a = rospy.ServiceProxy('walk_service', WalkService)
        a(n, step, side, ang)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def motion_client(motion_id):
    rospy.wait_for_service('motion_service')
    try:
        a = rospy.ServiceProxy('motion_service', MotionService)
        a(motion_id)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def servos_client(names, positions):
    rospy.wait_for_service('servo_service')
    try:
        servos_service = rospy.ServiceProxy('servo_service', ServoService)
        servos_service(names, positions)
    except rospy.ServiceException as e:
        print("Service call failed:", e)



if __name__ == "__main__":
    num = 10
    stepLength = 36
    sideLength = 0
    rotation = 0.0
    #while(True):
    #   walk_client(True, stepLength, sideLength, rotation)
    #    time.sleep(1)
    # walk_client(True, stepLength, 0, 0.0)
    # time.sleep(5)
    # walk_client(False, stepLength, 0, 0.0)
    motion_client('jump')

        
