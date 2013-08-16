#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('minimal')
    pub = rospy.Publisher('chatter', String)
    start = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < 10:
        log = "hello world %s" % rospy.get_time()
        rospy.loginfo(log)
        pub.publish(String(log))
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
