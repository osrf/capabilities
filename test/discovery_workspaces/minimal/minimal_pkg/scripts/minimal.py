#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def minimal():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('minimal')
    start = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < 10:
        str = "hello world %s" % rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        minimal()
    except rospy.ROSInterruptException:
        pass
