#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from tf.transformations import *
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from enum import Enum


class RelativePosition(Enum):
    BEFORE = 1
    BETWEEN = 2
    AFTER = 3


def publish_goals():
    # Read config file and publish goals as MarkerArray
    pub_goals = rospy.Publisher('/waypoints', MarkerArray, queue_size=10)
    pub_goals_line = rospy.Publisher('/waypoints_line', Marker, queue_size=10)
    waypoints_list = rospy.get_param("/goals")
    max_markers = len(waypoints_list)
    marker_array_msg = MarkerArray()
    for i in range(max_markers):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = Pose(Point(waypoints_list[i]["x"], waypoints_list[i]["y"], waypoints_list[i]["z"]),
                           Quaternion(1, 0, 0, 0))
        marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.8)
        marker.scale = Vector3(0.5, 0.5, 0.5)
        marker.frame_locked = False
        marker.ns = "path_planner"
        marker_array_msg.markers.append(marker)

    marker_msg = Marker()
    marker_msg.header.frame_id = "map"
    marker_msg.id = max_markers + 1
    marker_msg.type = Marker.LINE_STRIP
    marker_msg.action = Marker.ADD
    marker_msg.color = ColorRGBA(0.0, 0.0, 0.5, 0.8)
    marker_msg.frame_locked = False
    marker_msg.ns = "path_planner"
    for i in marker_array_msg.markers:
        marker_msg.points.append(i.pose.position)
    marker_msg.scale = Vector3(0.1, 0.1, 0.1)

    pub_goals_line.publish(marker_msg)
    pub_goals.publish(marker_array_msg)


class Robot(object):
    def __init__(self):
        self.sub = rospy.Subscriber("/waypoints", MarkerArray, self.callback)
        self.pub = rospy.Publisher('/robot', PoseStamped, queue_size=10)
        self.pose = PoseStamped()
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = "map"
        start_x = rospy.get_param("/start_pose_x")
        start_y = rospy.get_param("/start_pose_y")
        start_z = rospy.get_param("/start_pose_z")
        self.pose.pose.position = Point(start_x, start_y, start_z)
        self.pose.pose.orientation = Quaternion(1.0, 0.0, 0.0, 0.0)
        self.goals = []
        self.current_goal = -1
        self.acceptable_deviation = rospy.get_param("/acceptable_deviation")
        self.is_arrived = False

    def update(self):
        self.pub.publish(self.pose)

    def set_goals(self, goal):
        self.goals.append(goal)

    @staticmethod
    def dot_product( a, b):
        return np.dot(np.transpose(a), b)

    @staticmethod
    def get_relative_pose_to_segment(rob, a, b):

        ab = [None, None, None]
        ab[0] = b[0] - a[0]
        ab[1] = b[1] - a[1]
        ab[2] = b[2] - a[2]

        # vector B_rob
        be = [None, None, None]
        be[0] = b[0] - rob[0]
        be[1] = b[1] - rob[1]
        be[2] = b[2] - rob[2]

        # vector A_robE
        ae = [None, None, None]
        ae[0] = a[0] - rob[0]
        ae[1] = a[1] - rob[1]
        ae[2] = a[2] - rob[2]

        # rob_a = dot product between rob and a
        rob_a = Robot.dot_product(ab, ae)
        # rob_b = dot product between rob and b
        rob_b = Robot.dot_product(ab, be)
        if rob_a >= 0:
            return RelativePosition.AFTER
        elif rob_b < 0:
            return RelativePosition.BEFORE
        else:
            return RelativePosition.BETWEEN

    def check_if_goal_achieved(self):
        if (abs(self.pose.pose.position.x - self.goals[self.current_goal].x) < self.acceptable_deviation and
                abs(self.pose.pose.position.y - self.goals[self.current_goal].y) < self.acceptable_deviation and
                abs(self.pose.pose.position.z - self.goals[self.current_goal].z) < self.acceptable_deviation):
            if self.current_goal < (len(self.goals) - 1):
                self.current_goal = self.current_goal + 1
            else:
                self.is_arrived = True

    @staticmethod
    def vec_calc(start, end):
        x = end.x - start.x
        y = end.y - start.y
        z = end.z - start.z
        val = np.sqrt((x ** 2) + (y ** 2) + (z ** 2))
        return x / val, y / val, z / val

    def update_orient(self, segment_start, segment_end):
        x, y, z = Robot.vec_calc(segment_start, segment_end)

        roll = 0
        pitch = math.asin(-z)
        yaw = math.atan2(y, x)

        p = quaternion_from_euler(roll, pitch, yaw)

        self.pose.pose.orientation = Quaternion(*p)

    def update_pose(self, goal):
        print("update position ")
        x, y, z = Robot.vec_calc(self.pose.pose.position, goal)

        new_x = 0.1 * x
        new_y = 0.1 * y
        new_z = 0.1 * z

        self.pose.pose.position.x = self.pose.pose.position.x + new_x
        self.pose.pose.position.y = self.pose.pose.position.y + new_y
        self.pose.pose.position.z = self.pose.pose.position.z + new_z

    @staticmethod
    def euclidian_distance(start, end):
        x = end.x - start.x
        y = end.y - start.y
        z = end.z - start.z
        return np.sqrt((x ** 2) + (y ** 2) + (z ** 2))

    def get_closest_goal(self):
        max_dist = float("inf")
        closest_goal = -1
        for i in range(len(self.goals)):
            dist = Robot.euclidian_distance(self.pose.pose.position, self.goals[i])
            print("dist = ", dist)
            if dist < max_dist:
                max_dist = dist
                closest_goal = i
        return closest_goal

    def execute_path_planning(self):
        # first iteration
        if self.current_goal == -1:
            self.current_goal = self.get_closest_goal()
            # current goal is last point
            if self.current_goal != (len(self.goals) - 1):
                rob = np.array((self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z))
                a = np.array((self.goals[self.current_goal].x, self.goals[self.current_goal].y, self.goals[self.current_goal].z))
                b = np.array((self.goals[self.current_goal + 1].x, self.goals[self.current_goal + 1].y, self.goals[self.current_goal + 1].z))
                relative_position = Robot.get_relative_pose_to_segment(rob, a, b)
                # if closest goal is A but in segment AB
                if relative_position == RelativePosition.BETWEEN:
                    self.current_goal += 1
        # rest of iterations
        else:
            self.check_if_goal_achieved()

        # update position
        self.update_pose(self.goals[self.current_goal])
        # update orientation
        # for first point set heading towards point
        if self.current_goal == 0:
            self.update_orient(self.goals[0], self.goals[1])
        # for latest point set heading of last segment
        elif self.current_goal == (len(self.goals) - 1):
            self.update_orient(self.goals[len(self.goals) - 2], self.goals[len(self.goals) - 1])
        # for the rest of cases set heading of current segment
        else:
            self.update_orient(self.goals[self.current_goal - 1], self.goals[self.current_goal])

        print("execute path planning 2")

    def callback(self, data):
        self.update()
        self.goals = []
        print("\nCurrent position is = \n", self.pose.pose.position)
        for i in data.markers:
            self.set_goals(i.pose.position)
        if not self.is_arrived:
            self.execute_path_planning()
            i = self.get_closest_goal()
            print("Closest goal is ", i)


def path_plan():
    rospy.init_node('path_planner')
    rate = rospy.Rate(1)  # 10hz

    Robot().update()
    while not rospy.is_shutdown():
        publish_goals()
        rate.sleep()


if __name__ == '__main__':
    try:
        path_plan()
    except rospy.ROSInterruptException:
        pass
