#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def talker():
    pub = rospy.Publisher('chatter', Float32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        scan = Float32MultiArray(data=[1,2,3])
        print(scan)
        pub.publish(scan)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
