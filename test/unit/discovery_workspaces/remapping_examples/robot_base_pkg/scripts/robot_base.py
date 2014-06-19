#!/usr/bin/env python
import random
import time

import rospy

from std_msgs.msg import Float32


def main():
    rospy.init_node('robot_base')
    pub_front = rospy.Publisher('robot/front/distance', Float32, queue_size=1000)
    pub_rear = rospy.Publisher('robot/rear/distance', Float32, queue_size=1000)
    while not rospy.is_shutdown():
        msg = Float32(random.choice([1, 2, 3, 4]))
        rospy.loginfo("Sending distance: " + str(msg))
        pub_front.publish(msg)
        pub_rear.publish(msg)
        time.sleep(1)

if __name__ == '__main__':
    main()
