#! /usr/bin/env python

from math import atan, fabs, pi, pow, sqrt

import rospy
import actionlib
from geometry_msgs.msg import Twist
#from turtlesim.msg import Pose

from ekumen_turtle_commander.msg import Point2D, PolygonAction, PolygonFeedback, PolygonResult, Pose, Status
from ekumen_turtle_commander.srv import Pause, PauseResponse, Speed, SpeedResponse

class PolygonActionServer(object):
    _linear_error_tolerance = 0.0001
    _angular_error_tolerance = 0.0005
    _angular_scale = 6.0
    _linear_scale = 6.0

    def __init__(self, name):
        self._action_name = name

    def start(self):
        self._feedback = PolygonFeedback()
        self._result = PolygonResult()
        self._status = None
        self._as = actionlib.SimpleActionServer(self._action_name, PolygonAction, auto_start=False)
        self._as.register_goal_callback(self.goal_callback)
        self._velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self._pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, self.control_callback)
        self._pause_service = rospy.Service(self._action_name + "/pause", Pause, self.set_pause_callback)
        self._speed_service = rospy.Service(self._action_name + "/speed", Speed, self.set_speed_callback)
        self._as.start()

    def set_pause_callback(self, pause):
        if pause.pause:
            self._status.drawing_status = Status.PAUSED
        elif self._status.drawing_status == Status.PAUSED:
            self._status.drawing_status = Status.RESUMING
        return PauseResponse(last_pose=self._status.last_pose)

    def set_speed_callback(self, speed):
        response = SpeedResponse(previous_linear_speed=self._linear_scale, previous_angular_speed=self._angular_scale)
        self._angular_scale = speed.angular_speed
        self._linear_scale = speed.linear_speed
        return response

    # TODO: move elsewhere
    def distance(self, point1, point2):
        return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2))

    # TODO: move elsewhere
    def normalize_angle(self, theta):
        angle = theta % (2 * pi)
        return angle if angle < pi else angle - 2 * pi

    # TODO: move elsewhere
    def slope(self, point1, point2):
        denominator = (point1.x - point2.x)
        if denominator == 0.0:
            return None # float("inf")?
        else:
            return (point1.y - point2.y) / denominator

    # TODO: move elsewhere
    def theta_from_y0(self, point1, point2):
        slope = self.slope(point1, point2)
        if slope is None:
            angle = pi / 2
        else:
            angle = atan(slope)
        if point1.x > point2.x:
            angle += pi
        elif point1.y > point2.y:
            angle += pi * 2
        return angle

    # TODO: Research NumPy (or similar): https://docs.scipy.org/doc/numpy/reference/routines.array-manipulation.html
    # TODO: It would probably make sense to move this to the Status class. Figure this out.
    # This method relativises the (unreached) points from the current turtle position.
    # To use absolute coordinates, just do:
    #     self._status.absolute_points = self._goal.polygon.points
    def init_absolute_points(self, pose):
        goal_points = self._goal.polygon.points
        abs_points = self._status.absolute_points
        last_index = self._status.last_point_touched
        edge_travelled_distance = self._status.current_edge_travelled_distance

        if abs_points == []:
            abs_points += [Point2D()] * (len(goal_points) + 1)

        abs_p1 = abs_points[last_index]
        p1 = goal_points[last_index]
        p2 = goal_points[last_index + 1]
        adj = p2.x - p1.x
        opp = p2.y - p1.y
        if edge_travelled_distance == 0.0:
            shift = Point2D(x=0.0, y=0.0)
        else:
            if adj == 0.0:
                shift = Point2D(x=0.0, y=-edge_travelled_distance)
            elif opp == 0.0:
                shift = Point2D(x=-edge_travelled_distance, y=0.0)
            else:
                hyp = sqrt(pow(opp, 2) + pow(adj, 2))
                shift = Point2D(x=-edge_travelled_distance * adj / opp, y=-edge_travelled_distance * opp / hyp)
        abs_p1.x = pose.x + shift.x
        abs_p1.y = pose.y + shift.y
        poly_shift = Point2D(x=abs_p1.x - p1.x, y=abs_p1.y - p1.y)

        for index, goal_point in enumerate(goal_points[last_index + 1:], last_index + 1):
            abs_points[index] = Point2D(x=goal_point.x + poly_shift.x, y=goal_point.y + poly_shift.y)
        abs_points[-1] = Point2D(x=goal_points[0].x + poly_shift.x, y=goal_points[0].y + poly_shift.y)

    # inspired in turtlesim_actionlib tutorial package
    def compute_velocity_command(self, pose):
        distances = self._status.distances_from_origin
        last_index = self._status.last_point_touched
        next_point = self._status.absolute_points[last_index + 1]
        current_edge_length = distances[last_index + 1] - distances[last_index]

        # compute the distance and theta differentials for the edge
        theta_diff = self.normalize_angle(self.theta_from_y0(pose, next_point) - pose.theta)
        distance_diff = self.distance(pose, next_point)

        self._status.percentage_completed = 100 * (distances[last_index + 1] - distance_diff) / self._status.total_distance

        velocity = Twist()
        # adjust velocity
        if fabs(theta_diff) > self._angular_error_tolerance:
            velocity.linear.x = 0.0
            velocity.angular.z = self._angular_scale * theta_diff
        elif distance_diff > self._linear_error_tolerance:
            self._status.current_edge_travelled_distance = current_edge_length - distance_diff
            velocity.linear.x = self._linear_scale * distance_diff
            velocity.angular.z = 0.0
        else:
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self._status.current_edge_travelled_distance = 0.0
            self._status.last_point_touched += 1
            rospy.loginfo('%s: Drawn edge %i of %i.' % (self._action_name, self._status.last_point_touched, self._status.total_points))

        return velocity

    def control_callback(self, pose):
        if not self._as.is_active() or self._status is None or self._status.drawing_status == Status.PAUSED:
            return
        elif self._status.last_point_touched < self._status.total_points:
            distances = self._status.distances_from_origin
            if self._status.drawing_status == Status.RESUMING:
                self.init_absolute_points(pose)
                self._status.drawing_status = Status.ACTIVE
            self._status.last_pose = pose
            command = self.compute_velocity_command(pose)
            self._as.publish_feedback(PolygonFeedback(status=self._status))
            self._velocity_publisher.publish(command)
        else:
            self._as.set_succeeded(PolygonResult(x=pose.x, y=pose.y, theta=pose.theta))
            rospy.loginfo('%s: JOB FINISHED: Polygon of %i edges.' % (self._action_name, self._status.total_points))

    def calculate_distances(self, goal_points):
        total_distance = 0.0
        last_point = goal_points[0]
        distances = [0.0]
        for goal_point in goal_points[1:]:
            total_distance += self.distance(last_point, goal_point)
            distances.append(total_distance)
            last_point = goal_point
        total_distance += self.distance(last_point, goal_points[0])
        distances.append(total_distance)
        return (distances, total_distance)

    def goal_callback(self):
        self._goal = goal = self._as.accept_new_goal()

        (distances_from_origin, total_distance) = self.calculate_distances(goal.polygon.points)
        self._status = Status(absolute_points=[],
                              distances_from_origin=distances_from_origin,
                              total_points=len(goal.polygon.points),
                              total_distance=total_distance,
                              last_point_touched=0,
                              current_edge_travelled_distance=0.0,
                              percentage_completed=0.0,
                              drawing_status=Status.RESUMING)

        rospy.loginfo('%s: NEW GOAL: Polygon of %i edges.' % (self._action_name, self._status.total_points))
        rospy.loginfo('%s: Drawing edge 1 of %i.' % (self._action_name, self._status.total_points))

if __name__ == '__main__':
    rospy.init_node('polygon')
    server = PolygonActionServer(rospy.get_name())
    server.start()
    rospy.spin()

