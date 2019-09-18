#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import RADAR as RD
import math

pi = math.pi


def talker():
    pub = rospy.Publisher('radar_info', String, queue_size=10)
    rospy.init_node('RADAR', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass