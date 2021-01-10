import rospy
import actionlib
import tf2_ros
from multirobot_coverage.msg import *
from nav_msgs.msg import Path, Odometry
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist



if __name__ == "__main__":
  rospy.init_node("action_client")
  
  action_client = actionlib.SimpleActionClient("/robot1/path_follower", PathFollowAction)
  rospy.loginfo("Waiting for action server to come up...")
  action_client.wait_for_server()
  rospy.loginfo("Action server is active")
  path_arr = [(48.3, 24), (55.2, 23.7), (60.0, 18.0)]
  path = Path()
  path.header.frame_id = "map"
  path.header.stamp = rospy.Time.now()
  j = 0
  for x, y in path_arr:
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.seq = j
    j += 1
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.w = 1
    path.poses.append(p)
  
  goal = PathFollowGoal()
  goal.path = path
  action_client.send_goal(goal)
  print("Goal is send")
  action_client.wait_for_result()

  result = action_client.get_result()

  print("Result:", result)