#!/usr/bin/env python
PKG='ekumen_turtle_commander'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys
import unittest

from ekumen_turtle_commander.scripts import PolygonActionServer
from ekumen_turtle_commander.msg import Point2D, PolygonAction, PolygonFeedback, PolygonResult, Pose, Status

# TODO: Figure this out. Make tests work.

class TestPolygonActionServer(unittest.TestCase):
    def test_distance_of_horizontal_rightbound_line_is_positive(self):
        distance = PolygonActionServer('').distance(Point2D(x=1.0, y=1.0), Point2D(x=3.0, y=1.0))
        print(">>>>> distance =", distance)
        self.assertEquals(2.0, distance)

    def test_distance_of_vertical_upbound_line_is_positive(self):
        distance = PolygonActionServer('').distance(Point2D(x=-1.0, y=-1.0), Point2D(x=-1.0, y=3.0))
        print(">>>>> distance =", distance)
        self.assertEquals(4.0, distance)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_polygon_server', TestPolygonActionServer)
