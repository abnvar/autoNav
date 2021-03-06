#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import XVLidar
import pybreezyslam
import numpy as np
import time

from pathPlanning import *
from motionPlanning import *

pixelSpan = 800
distSpan = 40

class Scheduler():
    def __init__(self, repeat_time):
        self.time_init = -np.inf
        self.repeat_time = repeat_time

    def allowed(self):
        if time.time() - self.time_init > self.repeat_time:
            self.time_init = time.time()
            return 1
        else:
            return 0

def callback(scanList):
    global trajectory
    def slamUpdate(scan):
        slam.update(scan)

        x, y, theta = slam.getpos()
        slam.getmap(mapbytes)

        myMap = np.array(list(mapbytes)).reshape((800,800))
        myMap = myMap.astype(np.uint8)

        return myMap, x, y, theta

    scanList = list(scanList.data)
    destPos = Pos(400, 620)
    mapbytes = bytearray(pixelSpan*pixelSpan)

    myMap, x, y, theta = slamUpdate(scanList)

    updatedMap, canvas = prepareMap(myMap)
    startPos = Pos(int(y*pixelSpan/distSpan/1000), int(x*pixelSpan/distSpan/1000))

    if trajTimer.allowed():
        print(1)
        finder = PathFinder(startPos, destPos, updatedMap, canvas, 1)
        trajectory, _ = finder.shortestPath()

    # cv2.imshow('win', finder.canvas)
    # cv2.waitKey(1)

    ######

    if startPos.dist(destPos) < 10:
        print('Destination Reached.')
    else:
        if trajectory != None:
            for pt in trajectory:
                cv2.circle(canvas, (pt.h, pt.v), 1, (0, 0, 255), 1)

            planner = MotionPlanner(trajectory, startPos, theta, canvas)
            vl, vr = planner.getVelocities()

            cv2.circle(canvas, (int(planner.extPos.h), int(planner.extPos.v)), 1, (255, 0, 0), 1)
            cv2.circle(canvas, (destPos.h, destPos.v), 3, (255, 255, 0), 1)
            cv2.line(canvas, (int(planner.extPos.h), int(planner.extPos.v)), (startPos.h, startPos.v), (255,0,0), 1)

            cv2.imshow('win', canvas)
            cv2.waitKey(1)

    pub = rospy.Publisher('velocities', Float32MultiArray, queue_size=10)
    pub.publish(Float32MultiArray(data = [vl, vr]))
    print('published')


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    lidar = XVLidar()
    slam = RMHC_SLAM(lidar, pixelSpan, distSpan)
    trajectory = None
    trajTimer = Scheduler(10) # seconds for next update
    listener()
