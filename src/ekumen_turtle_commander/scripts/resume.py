#! /usr/bin/env python

from __future__ import print_function
import sys

import rospy

from ekumen_turtle_commander.srv import Pause

def pause_client(pause):
    rospy.wait_for_service('/polygon/pause')

    try:
        service = rospy.ServiceProxy('/polygon/pause', Pause)
        return service(pause)
    except rospy.ServiceException, exception:
        print("Service call failed: %s" % exception, file=sys.stderr)

def usage():
    return "Usage: %s" % sys.argv[0]

if __name__ == "__main__":
    argv = rospy.myargv()
    if len(argv) != 1:
        print(usage(), file=sys.stderr)
        sys.exit(-1)
    print(pause_client(False))

