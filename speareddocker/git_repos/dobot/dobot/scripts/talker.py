#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import time
from math import sin, cos

def talker():
    pub = rospy.Publisher('geometry_pose', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2) # 10hz

    t = 0.0
    a = 10.0
    f = 0.5
    while not rospy.is_shutdown():
        
	msg = Pose()
        msg.position.x = a*cos(t*f) + 180
        msg.position.y = a*sin(t*f) + 150
        msg.position.z = 20
        pub.publish(msg)
        rate.sleep()
      	t += 0.5

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
