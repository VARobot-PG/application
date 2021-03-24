#!/usr/bin/env python

from dobot_msgs.srv import *
from std_msgs.msg import Bool
import converter
import time
import sys
import rospy

isIdle = False

def callback(data):
    global isIdle
    isIdle = data.data
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/Left_Dobot/idle", Bool, callback)

    # spin() simply keeps python from exiting until this node is stopped

def move_client(x, y, z):
    rospy.wait_for_service('/Left_Dobot/SetPTPCmd')
    #x,y,z = converter.convert_coordinates(x,y,z,0)
    
    try:
        move = rospy.ServiceProxy('/Left_Dobot/SetPTPCmd', SetPTPCmd)
        resp1 = move(0, x, y, z, 0, False)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    listener()
    a = -101
    while a<200 and not rospy.is_shutdown():
	
	if(isIdle==True):
	    rospy.loginfo(isIdle)
	    rospy.loginfo("ros info in the loop")
	    a+=10
	    move_client(a,220,-61)
            time.sleep(0.5)
	    
    move_client(200,0,50)
    time.sleep(0.5)
    #print "Points: %s , %s , %s"%(a, 180, 50)
