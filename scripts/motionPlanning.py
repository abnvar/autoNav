import numpy as np
from simple_pid import PID
from pathPlanning import Pos
import cv2

class MotionPlanner():
    def __init__(self, trajectory, currPos, theta, canvas, vMax = 2):
        self.trajectory = np.array(trajectory)
        self.currPos = currPos
        self.theta = theta
        self.extPos = currPos + Pos(-15*np.cos(theta*np.pi/180), 15*np.sin(theta*np.pi/180))
        self.vMax = vMax
        self.pid = PID(1, 0.1, 0.5, setpoint=0)
        self.pid.sample_time = 0.01
        self.canvas = canvas

    def getClosestPt(self):
        minDist = np.inf
        cPt = self.trajectory[0]
        cIdx = 0
        for i in range(len(self.trajectory)):
            pt = self.trajectory[i]
            if self.extPos.dist(pt) < minDist:
                minDist = self.extPos.dist(pt)
                cPt = pt
                cIdx = i

        # cv2.circle(self.canvas, (self.trajectory[cIdx].h, self.trajectory[cIdx].v), 2, (0, 0, 255), 2)
        # cv2.circle(self.canvas, (self.trajectory[i].h, self.trajectory[i].v), 2, (255, 0, 0), 2)

        if cIdx > 10:
            side = 1 if (self.trajectory[cIdx] - self.trajectory[cIdx-10]).cross(self.extPos - self.trajectory[cIdx-10]) > 0 else -1
        else:
            side = 1 if (self.trajectory[cIdx+10] - self.trajectory[cIdx]).cross(self.extPos - self.trajectory[cIdx]) > 0 else -1

        return cPt, side, minDist

    def getVelocities(self):
        closestPt, side, minDist = self.getClosestPt()
        v = side*minDist

        vMax = self.vMax

        control = self.pid(v)

        vl = vMax/1 - control * 0.3
        vl = np.clip(vl, -vMax, vMax)
        vr = vMax/1 + control * 0.3
        vr = np.clip(vr, -vMax, vMax)

        return vl, vr


if __name__ == '__main__':
    trajectory = [Pos(1,1), Pos(0,5), Pos(0,10), Pos(0,20), Pos(100,500), Pos(100,600), Pos(300,500), Pos(600,600), Pos(800,800)]
    currPos = Pos(1,1)
    canvas = np.zeros((800,800))
    for theta in np.linspace(0,360,10):
        planner = MotionPlanner(trajectory, currPos, theta, canvas)
        vel = planner.getVelocities()
