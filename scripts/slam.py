import time

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import XVLidar
import pybreezyslam
import numpy as np
import cv2
import matplotlib.pyplot as plt

from pathPlanning import *
from motionPlanning import *

import vrep # access all the VREP elements

def scan360(clientID, servo):
    scan = []
    nScans = 72
    for i in np.linspace(0, 2*np.pi, nScans):
        e = vrep.simxSetJointPosition(clientID, servo, i, vrep.simx_opmode_blocking)
        val = 0
        e, _, val, _, _ = vrep.simxReadProximitySensor(clientID, sensor, vrep.simx_opmode_blocking)
        for i in range(360//nScans):
            scan.append(val[2] if abs(val[2]) < 100 else 100)
    # print(scan)s


    for i in range(len(scan)):
        # if scan[i] == 100:
        #     scan[i] = 50000
        # else:
        scan[i] *= 5000

    return scan


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


def scan(servo, scanFunc):
    while True:
        scan = scanFunc(clientID, servo)
        if len(scan) == 360:
            break
    slam.update(scan)

    x, y, theta = slam.getpos()
    slam.getmap(mapbytes)
    # print(list(mapbytes))
    myMap = np.array(list(mapbytes)).reshape((800,800))
    myMap = myMap.astype(np.uint8)

    # print(x,y, theta)
    coords[0].append(x)
    coords[1].append(y)
    thetas.append(theta)
    return myMap


def actuate(clientID, lMotor, rMotor, lVel, rVel):
    e = vrep.simxSetJointTargetVelocity(clientID, lMotor, lVel, vrep.simx_opmode_blocking)
    e = vrep.simxSetJointTargetVelocity(clientID, rMotor, rVel, vrep.simx_opmode_blocking)


if __name__ == '__main__':
    pixelSpan = 800
    distSpan = 40

    lidar = XVLidar()
    slam = RMHC_SLAM(lidar, pixelSpan, distSpan)
    coords = [[],[]]
    thetas = []
    mapbytes = bytearray(pixelSpan*pixelSpan)

    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # start a connection


    if clientID!=-1:
        print ("Connected to remote API server")

        e, servo = vrep.simxGetObjectHandle(clientID,"Servo", vrep.simx_opmode_blocking)
        # e, sensor = vrep.simxGetObjectHandle(clientID,"mainIR", vrep.simx_opmode_blocking)
        e, lMotor = vrep.simxGetObjectHandle(clientID,"dr20_leftWheelJoint_", vrep.simx_opmode_blocking)
        e, rMotor = vrep.simxGetObjectHandle(clientID,"dr20_rightWheelJoint_", vrep.simx_opmode_blocking)
        laserScannerHandle = vrep.simxGetObjectHandle(clientID, "LaserScanner_2D", vrep.simx_opmode_blocking)

        myMap = scan(servo, lidarScan360)

        updatedMap, canvas = prepareMap(myMap)
        startPos = Pos(int(coords[1][-1]*pixelSpan/distSpan/1000), int(coords[0][-1]*pixelSpan/distSpan/1000))
        theta = thetas[-1]
        destPos = Pos(400, 650)
        finder = PathFinder(startPos, destPos, updatedMap, canvas, 1)
        trajectory, _ = finder.shortestPath()

        cv2.imshow('win', finder.canvas)
        cv2.waitKey(1)

        planner = MotionPlanner(trajectory, startPos, theta, canvas)

        ######

        # actuate(lMotor, rMotor, 2.5, 2)

        time_init = time.time()
        trajectory_saved = None
        lastTrajUpdate = -10
        while time.time() - time_init < 150:
            map1 = scan(servo, lidarScan360)
            updatedMap, canvas = prepareMap(map1)

            startPos = Pos(int(coords[1][-1]*pixelSpan/distSpan/1000), int(coords[0][-1]*pixelSpan/distSpan/1000))
            theta = thetas[-1]
            destPos = Pos(400, 620)

            if startPos.dist(destPos) < 10:
                print('Destination Reached.')
                break

            if time.time() - lastTrajUpdate > 10:
                finder = PathFinder(startPos, destPos, updatedMap, canvas, 1)
                try:
                    trajectory, pathCost = finder.shortestPath()
                    for pt in trajectory:
                        if updatedMap[pt.v][pt.h] == 0:
                            trajectory = None
                            continue
                    if len(trajectory) < 1000:
                        trajectory_saved = trajectory
                        minPathCost = pathCost/len(trajectory)
                    else:
                        trajectory = trajectory_saved

                    lastTrajUpdate = time.time()
                except:
                    trajectory = trajectory_saved

            if trajectory != None:
                for pt in trajectory:
                    cv2.circle(canvas, (pt.h, pt.v), 1, (0, 0, 255), 1)
                print(pathCost)

                planner = MotionPlanner(trajectory, startPos, theta, canvas)
                vl, vr = planner.getVelocities()
                actuate(clientID, lMotor, rMotor, vl, vr)

                cv2.circle(canvas, (int(planner.extPos.h), int(planner.extPos.v)), 1, (255, 0, 0), 1)
                cv2.circle(canvas, (destPos.h, destPos.v), 3, (255, 255, 0), 1)
                cv2.line(canvas, (int(planner.extPos.h), int(planner.extPos.v)), (startPos.h, startPos.v), (255,0,0), 1)

                cv2.imshow('win', canvas)
                cv2.waitKey(1)
            else:
                actuate(clientID, lMotor, rMotor, 0.2, 0.2)

        actuate(clientID, lMotor, rMotor, 0, 0)

        status = vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
        print(status)
        time.sleep(1)
        status = vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
        print(status)

        vrep.simxFinish(-1)


    else:
        print("Not connected to remote API server")
