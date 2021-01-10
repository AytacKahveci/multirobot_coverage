import os
import sys
import math
import threading
from copy import deepcopy
import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import matplotlib.pyplot as plt 

import rospy
import actionlib
import tf2_ros

from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped, Twist
from multirobot_coverage.msg import *
from actionlib_msgs.msg import GoalStatus


class Robot:
  def __init__(self, name):
    self.name = name
    self.odom_sub = rospy.Subscriber("/" +name + "/odom", Odometry, self.odom_cb)
    self.cmd_pub = rospy.Publisher("/" + name + "/cmd_vel", Twist, queue_size=1)
    self.pose_pub = rospy.Publisher("/" + name + "/robot_pose", PoseStamped, queue_size=1)
    self.marker_pub = rospy.Publisher("/" + name + "/robot_marker", Marker, queue_size=1)
    self.buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.buffer)
    self.map_pose = PoseStamped()
    self.odom_pose = PoseStamped()

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

  def odom_cb(self, msg):
    pass

  def get_pose(self):
    """Get robot pose

    Returns:
      tuple(TransformStamped, TransformStamped):
        - TransformStamped: Map to base_footprint transform
        - TransformStamped: Odom to base_footprint transform
    """
    map_pose = None
    odom_pose = None
    try:
      map_tf = self.buffer.lookup_transform("map", self.name + "/base_footprint", rospy.Time(0), timeout=rospy.Duration(2.0))
      #odom_tf = self.buffer.lookup_transform("odom", self.name + "/base_footprint", rospy.Time(0), timeout=2.0)

      map_pose = PoseStamped()
      map_pose.pose.position.x = map_tf.transform.translation.x
      map_pose.pose.position.y = map_tf.transform.translation.y
      map_pose.pose.position.z = map_tf.transform.translation.z
      map_pose.pose.orientation.x = map_tf.transform.rotation.x
      map_pose.pose.orientation.y = map_tf.transform.rotation.y
      map_pose.pose.orientation.z = map_tf.transform.rotation.z
      map_pose.pose.orientation.w = map_tf.transform.rotation.w

      """ odom_pose = PoseStamped()
      odom_pose.pose.position.x = odom_tf.transform.translation.x
      odom_pose.pose.position.y = odom_tf.transform.translation.y
      odom_pose.pose.position.z = odom_tf.transform.translation.z
      odom_pose.pose.orientation.x = odom_tf.transform.rotation.x
      odom_pose.pose.orientation.y = odom_tf.transform.rotation.y
      odom_pose.pose.orientation.z = odom_tf.transform.rotation.z
      odom_pose.pose.orientation.w = odom_tf.transform.rotation.w """
   
    except tf2_ros.LookupException as e:
      rospy.logerr("Lookup exception: %s"%e)
    except tf2_ros.ExtrapolationException as e:
      rospy.logerr("Extrapolation exception: %s"%e)
    except Exception as e:
      rospy.logerr("Unhandled exception: %s"%e)

    return map_pose, odom_pose

  def command(self, msg):
    """Publish command velocity to robot

    Args:
      msg (Twist): Velocity command
    """
    self.cmd_pub.publish(msg)

  def run(self):
    """Main thread for robot

    """
    rate = rospy.Rate(10)
    first_msg = False
    while not rospy.is_shutdown():
      rospy.logwarn_throttle(4, "Rob thread")
      self.map_pose, self.odom_pose = self.get_pose()

      if self.map_pose and self.odom_pose:
        first_msg = True

      if first_msg:
        self.pose_pub.publish(self.map_pose)
        self.robot_marker.header.stamp = rospy.Time.now()
        self.robot_marker.pose = deepcopy(self.map_pose.pose)
        self.robot_marker.pose.orientation.w = 1
        self.marker_pub.publish(self.robot_marker) 
      rate.sleep()



class FuzzyLocalPlanner:
  def __init__(self, robot):
    self.robot = robot
    self.action_server = actionlib.SimpleActionServer("/" + robot.name + "/path_follower", PathFollowAction, execute_cb=self.execute_cb, auto_start=False)

    self.action_server.start()
    print("Server is started")

    self.feedback = PathFollowFeedback()
    self.result = PathFollowResult()

    self.global_plan = Path()
    self.local_plan = Path()

    self.look_ahead_distance = 1.0
    self.goal_dist_threshold = 0.1
    self.wheel_seperation = 0.3

    # Local planner path index
    self.robot_path_ind = 0
    self.goal_path_ind = None

    # Create Fuzzy Rules
    self.fuzzy_control = None
    self.fuzzy_control_sim = None
    self.create_fuzzy_rules()
    print("Fuzzy rules are created")


  def create_fuzzy_rules(self):
    distance = ctrl.Antecedent(np.arange(0.0, 2.0, 0.25), 'distance')
    distance['small']   = fuzz.trapmf(distance.universe, [-20, 0.0, 0.5, 1.0])
    distance['medium']  = fuzz.trimf(distance.universe, [0.7, 1.2, 1.5])
    distance['big']     = fuzz.trapmf(distance.universe, [1.3, 1.6, 2.0, 20])

    angle = ctrl.Antecedent(np.arange(-np.pi, np.pi, 0.2), 'angle')
    angle['nm'] = fuzz.trapmf(angle.universe, [-2*np.pi,-np.pi, -2*np.pi/3, -np.pi/3])
    angle['ns'] = fuzz.trimf(angle.universe, [-2*np.pi/3, -np.pi/3, 0])
    angle['z']  = fuzz.trimf(angle.universe, [-np.pi/3, 0, np.pi/3])
    angle['ps'] = fuzz.trimf(angle.universe, [0, np.pi/3, 2*np.pi/3])
    angle['pm'] = fuzz.trapmf(angle.universe, [np.pi/3, 2*np.pi/3, np.pi, 2*np.pi])
    
    wl = ctrl.Consequent(np.arange(0.0, 7.0, 0.1), 'wl')
    wl['zero'] = fuzz.trimf(wl.universe, [0, 0, 0])
    wl['small'] = fuzz.trimf(wl.universe, [0, 1.5, 3.0])
    wl['medium'] = fuzz.trimf(wl.universe, [2.5, 3.5, 5.0])
    wl['high'] = fuzz.trimf(wl.universe, [4.0, 6.0, 7.0])

    wr = ctrl.Consequent(np.arange(0.0, 7.0, 0.1), 'wr')
    wr['zero'] = fuzz.trimf(wl.universe, [0, 0, 0])
    wr['small'] = fuzz.trimf(wr.universe, [0, 1.5, 3.0])
    wr['medium'] = fuzz.trimf(wr.universe, [2.5, 3.5, 5.0])
    wr['high'] = fuzz.trimf(wr.universe, [4.0, 6.0, 7.0])

    """ distance.view()
    angle.view()
    wl.view()
    wr.view()
    plt.show() """

    rules = [
      ctrl.Rule(distance['small'] & angle['nm'], (wl['high'], wr['zero'])),
      ctrl.Rule(distance['small'] & angle['ns'], (wl['medium'], wr['small'])),
      ctrl.Rule(distance['small'] & angle['z'], (wl['small'], wr['small'])),
      ctrl.Rule(distance['small'] & angle['ps'], (wl['small'], wr['medium'])),
      ctrl.Rule(distance['small'] & angle['pm'], (wl['zero'], wr['high'])),

      ctrl.Rule(distance['medium'] & angle['nm'], (wl['high'], wr['zero'])),
      ctrl.Rule(distance['medium'] & angle['ns'], (wl['medium'], wr['small'])),
      ctrl.Rule(distance['medium'] & angle['z'], (wl['medium'], wr['medium'])),
      ctrl.Rule(distance['medium'] & angle['ps'], (wl['small'], wr['medium'])),
      ctrl.Rule(distance['medium'] & angle['pm'], (wl['zero'], wr['high'])),

      ctrl.Rule(distance['big'] & angle['nm'], (wl['high'], wr['zero'])),
      ctrl.Rule(distance['big'] & angle['ns'], (wl['high'], wr['medium'])),
      ctrl.Rule(distance['big'] & angle['z'], (wl['high'], wr['high'])),
      ctrl.Rule(distance['big'] & angle['ps'], (wl['medium'], wr['high'])),
      ctrl.Rule(distance['big'] & angle['pm'], (wl['zero'], wr['high'])),
    ]

    self.fuzzy_control = ctrl.ControlSystem(rules)
    self.fuzzy_control_sim = ctrl.ControlSystemSimulation(self.fuzzy_control)
    """ nodes = self.fuzzy_control_sim.ctrl.graph.nodes()
    out = ""
    for key, val in nodes.items():
        out += "{0} : {1}\n".format(key, val)
    print("Inputs: \n")
    print(out) """


  def set_plan(self, msg):
    """Set global plan

    msg (Path): Global path
    """
    self.global_plan = msg


  def get_local_plan(self, ind):
    """Get local plan with defined look-ahead distance

    Args:
      ind (int): Last index of the previous local plan in the global plan

    Returns:
      int: Last index of the local path in the global plan
      Path: Local path

    Raises:
      ValueError: if requested index is out of the global path range
    """
    size = len(self.global_plan.poses)
    if ind < 0 or ind >= size:
      raise ValueError("ind must be between 0 and %d"%size)
    
    start = self.global_plan.poses[ind].pose
    local_path = Path()
    found_ind = None
    for i in range(ind, size):
      candidate = self.global_plan.poses[i].pose
      dist = self.calc_distance(start, candidate)
      if dist >= self.look_ahead_distance:
        break
      else:
        local_path.poses.append(candidate)
        found_ind = i

    return found_ind, local_path


  @staticmethod
  def calc_distance(p1, p2):
    """Calculate Euclidean distance

    Args:
      p1(Pose): Pose1
      p2(Pose): Pose2

    Returns:
      float: Euclidean distance
    """
    return math.hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y)

  
  @staticmethod
  def calc_angle(p1, p2):
    """Calculate angle

    Args:
      p1(Pose): Pose1
      p2(Pose): Pose2

    Returns:
      float: Angle in range -pi, pi
    """
    angle = math.atan2(p2.position.y - p1.position.y, p2.position.x - p1.position.x)
    return angle


  @staticmethod
  def wrap_angle(angle):
    while True:
      if angle < 0.0:
        angle += 2.0*math.pi
      elif angle > 2.0 * math.pi:
        angle -= 2.0*math.pi
      else:
        break
    return angle


  def reset(self):
    """Reset local planner variables
    """
    self.robot_path_ind = 0
    self.goal_path_ind = None
    self.global_plan = Path()

  
  def goal_reached(self, robot_pose):
    """Check if goal reached

    Args:
      robot_pose (Pose): Current pose of the robot

    Returns:
      bool: True if goal reached, False otherwise
    """
    goal = self.global_plan.poses[-1].pose
    return self.calc_distance(robot_pose, goal) < self.goal_dist_threshold


  def get_twist(self, wl, wr):
    """Get twist

    Args:
      wl (float): Left wheel angular vel
      wr (float): Right wheel angular vel
    """
    v = (wl + wr) / 2.0
    w = (wr - wl) / self.wheel_seperation
    if w > 0.3:
      w = 0.3
    elif w < -0.3:
      w = -0.3
    if v > 1.0:
      v = 1
    elif v < -1.0:
      v = -1
    cmd_vel = Twist()
    cmd_vel.linear.x = v
    cmd_vel.angular.z = w
    return cmd_vel


  def calc_nearest_ind(self, robot_pose):
    """Calculate nearest index from local plan

    Return:
      int: Index
    """
    pass


  def compute_velociy_commands(self):
    """Compute velocity commands

    Returns:
      bool: True if velocity is computed, False otherwise
      Twist: Computed velocity command
    """
    map_pose, odom_pose = self.robot.get_pose()
    if map_pose == None:
      rospy.logerr("Robot pose could not retreived")
      return False, None


    if self.goal_reached(map_pose.pose):
      return True, self.get_twist(0.0, 0.0)


    if self.goal_path_ind == None:
      self.goal_path_ind, self.local_plan = self.get_local_plan(self.robot_path_ind)
    elif self.robot_path_ind == self.goal_path_ind:
      self.goal_path_ind, self.local_plan = self.get_local_plan(self.robot_path_ind)


    goal = self.global_plan.poses[self.robot_path_ind].pose
    dist = self.calc_distance(map_pose.pose, goal)
    angle = self.calc_angle(map_pose.pose, goal)

    if(dist < 0.3):
      self.robot_path_ind += 1
    
    self.fuzzy_control_sim.input['distance'] = dist
    self.fuzzy_control_sim.input['angle'] = angle

    self.fuzzy_control_sim.compute()

    wl = self.fuzzy_control_sim.output['wl']
    wr = self.fuzzy_control_sim.output['wr']

    rospy.logwarn("Distance: %f Angle: %f Wl: %f Wr: %f"%(dist, angle, wl, wr))
    rospy.logerr("Goal: (%f, %f) Robot: (%f, %f)"%(goal.position.x, goal.position.y, map_pose.pose.position.x, map_pose.pose.position.y))
    return True, self.get_twist(wl, wr)


  def sim(self):
    def create_point(x, y):
      p = Pose()
      p.position.x = x
      p.position.y = y
      p.orientation.w = 1.0
      return p
    
    path_raw = []
    for i in range(20):
      x = 10.0 + 10.0 * math.cos(2.0*math.pi*i/20.0)
      y = 10.0 + 10.0 * math.sin(2.0*math.pi*i/20.0)
      path_raw.append((x, y))

    path = [create_point(x, y) for x, y in path_raw]
    robot_pose = create_point(0, 0)

    ind = 0

    rate = rospy.Rate(10)
    start = rospy.Time.now()
    _heading = 0.0
    while(not rospy.is_shutdown()):
      if(ind >= len(path)):
        print("Goal Reached!")
        break

      goal = path[ind]
      dist = self.calc_distance(robot_pose, goal)
      angle = self.calc_angle(robot_pose, goal) - _heading
      angle = self.wrap_angle(angle)

      if dist < 0.1:
        ind+=1
      self.fuzzy_control_sim.input['distance'] = dist
      self.fuzzy_control_sim.input['angle'] = angle

      self.fuzzy_control_sim.compute()

      wl = self.fuzzy_control_sim.output['wl']
      wr = self.fuzzy_control_sim.output['wr']

      v = (wl + wr) * 0.5
      w = (wr - wl) / 0.3
      dt = (rospy.Time.now() - start).to_sec()
      start = rospy.Time.now()

      _heading += (w * dt)
      robot_pose.position.x += (v * dt * math.cos(_heading))
      robot_pose.position.y += (v * dt * math.sin(_heading))

      rospy.logwarn("Distance: %f Angle: %f Wl: %f Wr: %f"%(dist, angle, wl, wr))
      rospy.logerr("Goal: (%f, %f) Robot: (%f, %f)"%(goal.position.x, goal.position.y, robot_pose.position.x, robot_pose.position.y))

      rate.sleep()

  
  def execute_cb(self, goal):
    """Action server execute handler
    """
    print("Action server")
    loop_rate = rospy.Rate(10)
    
    self.set_plan(goal.path)
    while not rospy.is_shutdown():
      rospy.logwarn_throttle(2.0,"ExecCb")
      map_pose, odom_pose = self.robot.get_pose()
      if map_pose == None:
        rospy.logerr("Robot pose could not retreived")
        continue
      
      if self.goal_reached(map_pose.pose):
        # Robot completed the task
        self.result.success = True
        self.action_server.set_succeeded(self.result)
        rospy.logerr("Goal reached")
        return
      
      if self.action_server.is_preempt_requested():
        print("Preempt requested")
        self.result.success = False
        self.action_server.publish_feedback(self.feedback)
        self.action_server.set_preempted(result=self.result)
        return



      res, cmd_vel = self.compute_velociy_commands()
      
      if res:
        self.robot.command(cmd_vel)

      loop_rate.sleep()




if __name__ == "__main__":
  rospy.init_node("fuzzy_controller")

  robot = Robot("robot1")
  t = threading.Thread(target=robot.run)
  t.start()
  local_planner = FuzzyLocalPlanner(robot=robot)

  rospy.spin()