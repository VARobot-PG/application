#!/usr/bin/env python
import rospy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Bool

class PublisherSubscriber:
    current_value=True
    #listener below here

    def callback(self,data):
        if self.current_value != data.data:
            rospy.loginfo("I heard %s.It was %s before", data.data,self.current_value)
        self.current_value = data.data


    def listener(self):
        rospy.init_node('AddDetectedObjectServer')
        rospy.Subscriber("/Dobot_Loader/idle", Bool, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()





    #publisher below here
    def run(self):
        pub = rospy.Publisher('/Mock/Dobot_Loader/idle', GoalID, queue_size=10)
        rospy.init_node('AddDetectedObjectServer')
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.listener()
            my_message = GoalID()
            my_message.id=str(self.current_value)
            my_message.stamp.nsecs=int(self.current_value)
            my_message.stamp.secs=int(self.current_value)
            pub.publish(my_message)
            r.sleep()

if __name__ == "__main__":
    me = PublisherSubscriber()
    me.run()


