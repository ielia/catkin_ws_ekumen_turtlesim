#! /usr/bin/env python

from __future__ import print_function
import sys

import rospy

from ekumen_turtle_commander.srv import Speed

def speed_client(linear_speed, angular_speed):
    rospy.wait_for_service('/polygon/speed')

    try:
        service = rospy.ServiceProxy('/polygon/speed', Speed)
        return service(linear_speed, angular_speed)
    except rospy.ServiceException, exception:
        print("Service call failed: %s" % exception, file=sys.stderr)

def usage():
    return "Usage: %s <linear_speed> <angular_speed>" % sys.argv[0]

if __name__ == "__main__":
    argv = rospy.myargv()
    if len(argv) != 3:
        print(usage(), file=sys.stderr)
        sys.exit(-1)
    else:
        try:
            linear_speed = float(argv[1])
            angular_speed = float(argv[2])
        except:
            print(usage(), file=sys.stderr)
            sys.exit(-2)
    print(speed_client(linear_speed, angular_speed))

