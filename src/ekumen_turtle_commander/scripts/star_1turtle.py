#! /usr/bin/env python
#
# TODO: Explore this posibility in a server too...
#
# def draw_star(points, period, side_length):
#     assert_coprimes(points, period)
#     for i in range(points):
#         advance(turtle1, 1)
#     turtle.forward(side_length)
#     loginfo("finished")
#
# def draw_star_two_turtles(points, period, n_turtles, side_length):
#     assert_coprimes(points / 2, period / 2)
#     spawn(turtle=turtle2, point=period + 1)
#     for i in range(points / 2):
#         advance(turtle1, 1)
#         advance(turtle2, 2)
#     turtle1.forward(side_length)
#     loginfo("drawn edge %i of %i with turtle #%i" % (i, points, 1))
#     turtle2.forward(side_length)
#     loginfo("finished")
#
# def advance(turtle, tnum):
#     turtle.forward(side_length)
#     loginfo("drawn edge %i of %i with turtle #%i" % (i, points, tnum))
#     turtle.right(2 * pi * period / points)
#

from __future__ import print_function
import sys
from math import cos, pi, sin

import rospy
import actionlib

from ekumen_turtle_commander.msg import Point2D, Polygon2D, PolygonAction, PolygonGoal

def handle_feedback(feedback):
    print("Feedback:", feedback)

def coprime(a, b):
    while a * b != 0:
        if a > b:
            a %= b
        else:
            b %= a
    return max(a, b) == 1

def polygon_client(vertices, period, radius):
    client = actionlib.SimpleActionClient('polygon', PolygonAction)

    angle = 2 * pi / vertices
    points = [Point2D(x=0.0, y=0.0)]
    v = period
    while v != 0:
        x = (cos(angle * v) - 1) * radius
        y = sin(angle * v) * radius
        points.append(Point2D(x=x, y=y))
        v = (v + period) % vertices

    goal = PolygonGoal(polygon=Polygon2D(points=points))

    client.wait_for_server()
    client.send_goal(goal, feedback_cb=handle_feedback)
    client.wait_for_result()
    return client.get_result()

def usage():
    return """Usage: %s <#vertices> <period> <radius>
        #vertices = Number of vertices of the star to be drawn.
        period    = Skip period (i.e., vertices arranged clockwise are to be
                    connected every period+1).
        radius    = Radius of the star (float).

        Notes:
               * #vertices and period have to be coprime.
               * radius has to be greater than 1.0.
""" % sys.argv[0]

if __name__ == '__main__':
    argv = rospy.myargv()
    if len(argv) != 4:
        print(usage(), file=sys.stderr)
        sys.exit(-1)
    else:
        try:
            vertices = int(argv[1])
            period = int(argv[2])
            radius = float(argv[3])
        except:
            print(usage(), file=sys.stderr)
            sys.exit(-2)

    assert vertices > 1, "Number of vertices has to be greater than 1."
    assert period > 1, "Period has to be greater than 1."
    assert vertices > period, "Number of vertices has to be greater than the period."
    assert coprime(vertices, period), "Vertices and period are not coprime."
    assert radius > 1.0, "Radius has to be greater than 1"

    try:
        rospy.init_node('star_1turtle_py')
        result = polygon_client(vertices, period, radius)
        print("Result:", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

