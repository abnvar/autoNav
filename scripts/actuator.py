#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import vrep


def actuate(clientID, lMotor, rMotor, lVel, rVel):
    e = vrep.simxSetJointTargetVelocity(clientID, lMotor, lVel, vrep.simx_opmode_blocking)
    e = vrep.simxSetJointTargetVelocity(clientID, rMotor, rVel, vrep.simx_opmode_blocking)

def callback(data):
    vl, vr = data.data
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # start a connection
    if clientID != -1:
        print('actuated')
        e, lMotor = vrep.simxGetObjectHandle(clientID,"dr20_leftWheelJoint_", vrep.simx_opmode_blocking)
        e, rMotor = vrep.simxGetObjectHandle(clientID,"dr20_rightWheelJoint_", vrep.simx_opmode_blocking)
        actuate(clientID, lMotor, rMotor, vl/3, vr/3)
        vrep.simxFinish(-1)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('velocities', Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
