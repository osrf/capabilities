#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('slow_to_die')
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

    # Take at least 3 seconds to die
    t_start = time.time()
    while time.time() - t_start < 3.0:
        time.sleep(0.5)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

