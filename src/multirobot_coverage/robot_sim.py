#!/usr/bin/env python
import os
import math
import threading
from enum import Enum
from copy import deepcopy

import rospy
import actionlib
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from multirobot_coverage.msg import RobotState
from geometry_msgs.msg import Pose, PoseStamped
from multirobot_coverage.msg import *
from actionlib_msgs.msg import GoalStatus


class States(Enum):
  WAITING=0
  WORKING=1


class RobotSim:
  def __init__(self, name, init_pose):
    self.name = name
    self.action_server = actionlib.SimpleActionServer("/" + self.name + "/path_follower", PathFollowAction, execute_cb=self.execute_cb, auto_start=False)
    self.pose_pub = rospy.Publisher("/" + self.name + "/robot_pose", PoseStamped, queue_size=1)
    self.marker_pub = rospy.Publisher("/" + self.name + "/robot_marker", Marker, queue_size=1)
    self.action_server.start()

    self.feedback = PathFollowFeedback()
    self.result = PathFollowResult()

    self.robot_marker = Marker()
    self.robot_marker.header.frame_id = "map"
    self.robot_marker.id = 1
    self.robot_marker.type = Marker.CUBE
    self.robot_marker.action = Marker.ADD
    self.robot_marker.color.r = 1.0
    self.robot_marker.color.a = 1.0
    self.robot_marker.scale.x = 0.5
    self.robot_marker.scale.y = 0.5
    self.robot_marker.scale.z = 0.5

    self.current_pose = init_pose
    self.robot_pose_thread = threading.Thread(target=self.run, args=(10,))
    self.robot_pose_thread.start()


  def __del__(self):
    rospy.signal_shutdown()
    self.robot_pose_thread.join()


  def publish_robot_pose(self):
    p = PoseStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "map"
    p.pose = self.current_pose
    self.pose_pub.publish(p)


  def execute_cb(self, goal):
    """Action server execute handler
    """
    loop_rate = rospy.Rate(10)
    ind = 0
    self.current_pose = deepcopy(goal.path.poses[0].pose)
    while not rospy.is_shutdown():
      if ind >= (len(goal.path.poses)):
        # Robot completed the task
        self.result.success = True
        self.action_server.set_succeeded(self.result)
        return

      if self.action_server.is_preempt_requested():
        print("Preempt requested")
        self.result.success = False
        self.action_server.publish_feedback(self.feedback)
        self.action_server.set_preempted(result=self.result)
        return

      dist = self.calc_dist(self.current_pose, goal.path.poses[ind].pose)
      angle = self.calc_angle(self.current_pose, goal.path.poses[ind].pose)
      print("Current: ", self.current_pose)
      print("Target: ", goal.path.poses[ind].pose)
      print("Angle: ", angle)
      if dist > 0.40:
        print("Distance: ", dist)
        self.current_pose.position.x += dist * 0.4 * math.cos(angle)
        self.current_pose.position.y += dist * 0.4 * math.sin(angle)
      else:
        print("Next state")
        self.feedback.traversed.header.stamp = rospy.Time.now()
        self.feedback.traversed.header.frame_id = "map"
        self.feedback.traversed.poses.append(deepcopy(goal.path.poses[ind]))
        ind += 100

      self.robot_marker.header.stamp = rospy.Time.now()
      self.robot_marker.pose = deepcopy(self.current_pose)
      self.robot_marker.pose.orientation.w = 1
      
      self.action_server.publish_feedback(self.feedback)
      self.marker_pub.publish(self.robot_marker)    
      
      loop_rate.sleep()

  def run(self, rate):
    """Robot pose publisher thread
    """
    loop_rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
      self.publish_robot_pose()
      loop_rate.sleep()

  @staticmethod
  def calc_dist(p1, p2):
    return math.hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y)

  @staticmethod
  def calc_angle(p1, p2):
    return math.atan2(p2.position.y - p1.position.y, p2.position.x - p1.position.x)



if __name__ == "__main__":
  rospy.init_node("robot_sim")

  robot_name = rospy.get_param("~robot_name", "robot1")
  robot_id = rospy.get_param("~robot_id", 1)
  x_pos = rospy.get_param("~x_pos")
  y_pos = rospy.get_param("~y_pos")

  r1_pose = Pose()
  r1_pose.position.x = x_pos
  r1_pose.position.y = y_pos
  r1_pose.position.z = 0.0

  robot_sim = RobotSim(robot_name, r1_pose)

  rospy.spin()