#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def on_cmd_vel(msg):
    log = "Moving robot with linear x velocity '{0}' and angular yaw velocity '{1}'"
    rospy.loginfo(log.format(msg.linear.x, msg.angular.z))


def main():
    rospy.init_node('faux_base')
    rospy.Subscriber('cmd_vel', Twist, on_cmd_vel)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
