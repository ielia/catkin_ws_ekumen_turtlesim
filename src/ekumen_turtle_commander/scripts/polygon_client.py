#! /usr/bin/env python

from __future__ import print_function
import re
import sys

import rospy
import actionlib

from ekumen_turtle_commander.msg import Point2D, Polygon2D, PolygonAction, PolygonGoal

def handle_feedback(feedback):
    print("Feedback:", feedback)

def read_polygon_file(filename):
    points = []
    with open(filename, "r") as instream:
        nline = 1
        for line in instream:
            if not line.startswith("#"):
                coordinates = list(filter(None, re.split("\\s+", line)))
                if len(coordinates) > 0:
                    assert len(coordinates) == 2, "Format error on line %i: Each line must contain a single point, x and y coordinates separated by a space." % nline
                    try:
                        x = float(coordinates[0])
                        y = float(coordinates[1])
                    except:
                        print("Format error on line %i: Both components must be floating point numbers in decimal notation." % nline, file=sys.stderr)
                        sys.exit(-2)
                    points.append(Point2D(x=x, y=y))
            nline += 1
    return points

def polygon_client(filename):
    client = actionlib.SimpleActionClient('polygon', PolygonAction)
    client.wait_for_server()

    points = read_polygon_file(filename)

    goal = PolygonGoal(polygon=Polygon2D(points=points))

    client.send_goal(goal, feedback_cb=handle_feedback)
    client.wait_for_result()
    return client.get_result()

def usage():
    return "Usage: %s <filename>" % sys.argv[0]

if __name__ == '__main__':
    argv = rospy.myargv()
    if len(argv) != 2:
        print(usage(), file=sys.stderr)
        sys.exit(-1)
    filename = argv[1]
    try:
        rospy.init_node('polygon_client_py')
        result = polygon_client(filename)
        print("Result:", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
    except IOError:
        print("could not read", filename, file=sys.stderr)

