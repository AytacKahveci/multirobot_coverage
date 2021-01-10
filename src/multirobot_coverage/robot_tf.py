import math
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry


_wheel_seperation = 0.3
_x = 42.0
_y = 24.0
_heading = 0.0


class RobotTf:
  def __init__(self, name):
    self.name = name
    self.robot_pose = PoseStamped()
    self.robot_pose.header.frame_id = "map"
    self.robot_pose.pose.position.x = _x
    self.robot_pose.pose.position.y = _y
    self.robot_pose.pose.orientation.w = 1

    self.tf_broadcaster = tf2_ros.TransformBroadcaster()
  
    self.cmd_sub = rospy.Subscriber("/" + self.name + "/cmd_vel", Twist, self.cmd_cb)
    self.first_msg = False


  def cmd_cb(self, msg):
    global _heading
    if not self.first_msg:
      self.start = rospy.Time.now()
      dt = 0.1
    else:
      dt = (rospy.Time.now() - self.start).to_sec()
      self.start = rospy.Time.now()
    self.first_msg = True
    v = msg.linear.x
    w = msg.angular.z

    
    print(v, w, dt)
    _heading += (w * dt)
    self.robot_pose.pose.position.x += (v * dt * math.cos(_heading))
    self.robot_pose.pose.position.y += (v * dt * math.sin(_heading))

  def run(self):
    global _heading

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      q = tft.quaternion_from_euler(0, 0, _heading)
      self.robot_pose.pose.orientation.x = q[0]
      self.robot_pose.pose.orientation.y = q[1]
      self.robot_pose.pose.orientation.z = q[2]
      self.robot_pose.pose.orientation.w = q[3]

      tf_pose = TransformStamped()
      tf_pose.header.stamp = rospy.Time.now()
      tf_pose.header.frame_id = "map"
      tf_pose.child_frame_id = self.name + "/base_footprint"
      tf_pose.transform.translation.x = self.robot_pose.pose.position.x
      tf_pose.transform.translation.y = self.robot_pose.pose.position.y
      tf_pose.transform.translation.z = self.robot_pose.pose.position.z
      tf_pose.transform.rotation.x = self.robot_pose.pose.orientation.x
      tf_pose.transform.rotation.y = self.robot_pose.pose.orientation.y
      tf_pose.transform.rotation.z = self.robot_pose.pose.orientation.z
      tf_pose.transform.rotation.w = self.robot_pose.pose.orientation.w
      self.tf_broadcaster.sendTransform(tf_pose)

      """ tf_pose = TransformStamped()
      tf_pose.header.stamp = rospy.Time.now()
      tf_pose.header.frame_id = "odom"
      tf_pose.child_frame_id = self.name + "/base_footprint"
      tf_pose.transform.translation.x = self.robot_pose.pose.position.x
      tf_pose.transform.translation.y = self.robot_pose.pose.position.y
      tf_pose.transform.translation.z = self.robot_pose.pose.position.z
      tf_pose.transform.rotation.x = self.robot_pose.pose.orientation.x
      tf_pose.transform.rotation.y = self.robot_pose.pose.orientation.y
      tf_pose.transform.rotation.z = self.robot_pose.pose.orientation.z
      tf_pose.transform.rotation.w = self.robot_pose.pose.orientation.w
      self.tf_broadcaster.sendTransform(tf_pose) """

      rospy.loginfo_throttle(1, "Running")
      rate.sleep()


if __name__ == "__main__":
  rospy.init_node("robot_tf")
  
  robot_name = rospy.get_param("~robot_name", "robot1")
  robot_id = rospy.get_param("~robot_id", 1)
  robot_size = rospy.get_param("~robot_size", 0.5)
  
  rob_tf = RobotTf(robot_name)
  rob_tf.run()