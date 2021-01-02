import math
from enum import Enum
from copy import deepcopy

import rospy
import actionlib
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from multirobot_coverage.msg import RobotState
from geometry_msgs.msg import Pose, PoseStamped


class States(Enum):
  WAITING=0
  WORKING=1

path = Path
path_received = False
enable = False

def path_cb(msg):
  global path, path_received
  path = msg
  path_received = True

def enable_cb(msg):
  global enable
  enable = msg.data

def calc_dist(p1, p2):
  return math.hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y)

def calc_angle(p1, p2):
  return math.atan2(p2.position.y - p1.position.y, p2.position.x - p1.position.x)

def execute_cb(goal):
  loop_rate = rospy.Rate(10)
  ind = 0
  current_pose = deepcopy(goal.path.poses[0].pose)
  while not rospy.is_shutdown():
    feedback = ActionFeedback()
    result = ActionResult()
    loop_rate.sleep()

    if ind >= (len(path.poses)):
      # Robot completed the task
      result.success = True
      

if __name__ == "__main__":
  rospy.init_node("robot_sim")

  path_sub = rospy.Subscriber("/planned_path", Path, path_cb)
  enable_sub = rospy.Publisher("/enable_sub", Bool, enable_cb)
  marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size=1)
  completed_pub = rospy.Publisher("/completed", Bool, queue_size=1)
  action_server = actionlib.SimpleActionServer("/path_follower", Action, execute_cb=execute_cb)
  action_server.start()
  robot_state = States.WAITING
  robot_marker = Marker()

  loop_rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    if robot_state == States.WAITING:
      rospy.loginfo_throttle(1, "Waiting")
      if enable:
        continue
      # Robot is waiting for path
      if path_received:
        path_received = False
        robot_state = States.WORKING
        current_pose = deepcopy(path.poses[0].pose)
        ind = 1
    elif robot_state == States.WORKING:
      rospy.loginfo_throttle(1, "Working ind: %d"%(ind))
      # Robot is following the path
      if ind >= (len(path.poses)):
        # Robot completed the task
        robot_state = States.WAITING
        path = Path()
        continue
      dist = calc_dist(current_pose, path.poses[ind].pose)
      angle = calc_angle(current_pose, path.poses[ind].pose)
      print("Current: ", current_pose)
      print("Target: ", path.poses[ind].pose)
      print("Angle: ", angle)
      if dist > 0.40:
        print("Distance: ", dist)
        dp = min(dist, 0.40)
        current_pose.position.x += dist * 0.4 * math.cos(angle)
        current_pose.position.y += dist * 0.4 * math.sin(angle)
      else:
        print("Next state")
        ind += 5

      robot_marker.header.stamp = rospy.Time.now()
      robot_marker.header.frame_id = "map"
      robot_marker.id = 1
      robot_marker.type = Marker.CUBE
      robot_marker.action = Marker.ADD
      robot_marker.pose = deepcopy(current_pose)
      robot_marker.pose.orientation.w = 1
      robot_marker.color.r = 1.0
      robot_marker.color.a = 1.0
      robot_marker.scale.x = 0.5
      robot_marker.scale.y = 0.5
      robot_marker.scale.z = 0.5
      marker_pub.publish(robot_marker)
    
    loop_rate.sleep()