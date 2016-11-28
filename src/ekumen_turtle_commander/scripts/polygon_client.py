#! /usr/bin/env python

from __future__ import print_function

import rospy
import actionlib

from ekumen_turtle_commander.msg import Point2D, Polygon2D, PolygonAction, PolygonGoal

def handle_feedback(feedback):
    print("Feedback:", feedback)

def polygon_client():
    client = actionlib.SimpleActionClient('polygon', PolygonAction)
    client.wait_for_server()

    goal = PolygonGoal(polygon=Polygon2D(points=[Point2D(x=0.0, y=0.0), Point2D(x=1.0, y=0.0), Point2D(x=0.5, y=1.0)]))

    client.send_goal(goal, feedback_cb=handle_feedback)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('polygon_client_py')
        result = polygon_client()
        print("Result:", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

