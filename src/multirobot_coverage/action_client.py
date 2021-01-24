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
  path_arr = [(40, 28.1), (40, 27.1), (40, 26.1), (40, 25.1), (40, 24.3), (40, 23.1), (40, 22.1), (40, 21.1), (40, 20.1), (40, 17.3), (38.1, 16.4)]
  path_arr2 = [(38, 28.1), (38, 27.1), (38, 26.1), (38, 25.1), (38, 24.3), (38, 23.1), (38, 22.1), (38, 21.1), (38, 20.1), (38, 17.3), (38.1, 16.4)]

  path_arr = path_arr + path_arr2[::-1]
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