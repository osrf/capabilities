#!/usr/bin/env python

from math import sin

import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node('faux_navigation')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    angles = []
    while not rospy.is_shutdown():
        if not angles:
            angles = [sin(x) for x in range(100)]
        linear = 1.0
        angular = angles.pop()
        log = "Sending command velocity ({0}, {1})".format(linear, angular)
        rospy.loginfo(log)
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        pub.publish(msg)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
