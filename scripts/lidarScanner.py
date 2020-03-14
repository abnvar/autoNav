#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

import vrep # access all the VREP elements


def lidarScan360(clientID, servo):
    scan = []
    e = vrep.simxSetJointPosition(clientID, servo, -np.pi/2, vrep.simx_opmode_blocking)
    eData = vrep.simxGetStringSignal(clientID, "mySignal", vrep.simx_opmode_blocking)
    scan = vrep.simxUnpackFloats(eData[1])
    e = vrep.simxSetJointPosition(clientID, servo, np.pi/2, vrep.simx_opmode_blocking)
    eData = vrep.simxGetStringSignal(clientID, "mySignal", vrep.simx_opmode_blocking)
    data = vrep.simxUnpackFloats(eData[1])
    scan = scan[::-1]
    data = data[::-1]
    for x in data:
        scan.append(x)

    for i in range(len(scan)):
        scan[i] = min(50000, 5000*scan[i])

    while len(scan) > 360:
        scan.pop()
    # print(scan)
    return scan

def talker():
    pub = rospy.Publisher('chatter', Float32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.4) # hz

    # lidar = XVLidar()
    # vrep.simxFinish(-1) # just in case, close all opened connections
    servo = None
    while not rospy.is_shutdown():
        clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # start a connection
        if clientID!=-1:
            if servo is None:
                e, servo = vrep.simxGetObjectHandle(clientID,"Servo", vrep.simx_opmode_blocking)
                laserScannerHandle = vrep.simxGetObjectHandle(clientID, "LaserScanner_2D", vrep.simx_opmode_blocking)
            scanList = lidarScan360(clientID, servo)
            print ("Lidar Scan Complete.")
        else:
            print("Not connected to remote API server")

        vrep.simxFinish(-1)
        scanList = Float32MultiArray(data=scanList)
        # print(scanList)
        pub.publish(scanList)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
