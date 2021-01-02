#!/usr/bin/env python
import cv2
import time
import logging
from enum import Enum
from threading import Lock
from transitions import State, Machine

import rospy
import actionlib
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
from multirobot_coverage.msg import RobotState
from multirobot_coverage.msg import PathFollowAction, PathFollowActionFeedback, PathFollowActionResult, PathFollowGoal

from robot import Robot
from graph import Graph
from covarage_plan import Voronoi
from path_plan import PathPlan


class States(Enum):
  """FSM states
  """
  IDLE = 0
  CPP = 1
  WORKING = 2
  STOP = 3


class MCPP(object):
  """MCPP finite state machine class
  """

  def __init__(self, name, robot_id, robot_size):
    """Constructor

    Args:
      name (str): robot name which fsm belong to
      robot_id (int): unique robot id
      robot_size (float): robot footprint diameter
    """
    self.name = name
    self.robot_id = robot_id
    # robot object which fsm belongs to
    self.robot = Robot(self.robot_id, Pose(), 1.0, [0, 255, 0], robot_size)
    self.robots = {self.robot.robot_id: self.robot}
    self.prev_robots = dict()

    self.mutex = Lock()
    self.logger = logging.getLogger(__name__)
    self.logger.addHandler(logging.StreamHandler())
    self.logger.setLevel(logging.DEBUG)
    
    self.map_service = "/static_map"
    self.logger.info("Waiting for map service to come up...")
    rospy.wait_for_service(self.map_service)
    self.logger.info("Map service is active")
    self.graph = Graph(self.map_service)

    self.start_request = False
    self.stop_request = False
    self.start_sub = rospy.Subscriber("/" + self.name + "/start_request", Bool, self.start_cb)
    self.stop_sub = rospy.Subscriber("/" + self.name + "/stop_request", Bool, self.stop_cb)
    self.pose_sub = rospy.Subscriber("/" + self.name + "/robot_pose", PoseStamped, self.pose_cb)
    self.path_pub = rospy.Publisher("/" + self.name + "/planned_path", Path, queue_size=1, latch=True)
    self.state_sub = rospy.Subscriber("/robot_state", RobotState, self.robot_state_cb)
    self.state_pub = rospy.Publisher("/robot_state", RobotState, queue_size=1)

    self.current_pose = None

    self.robot_state = RobotState()
    self.robot_state.robot_id = self.robot.robot_id
    self.robot_state.working_state = States.IDLE
    self.robot_state.traversed_cells = []

    self.action_client = actionlib.SimpleActionClient("/" + self.name + "/path_follower", PathFollowAction)
    self.logger.info("Waiting for action server to come up...")
    self.action_client.wait_for_server()
    self.logger.info("Action server is active")
    
    self.cpp_path = Path()


  def __del__(self):
    pass


  def pose_cb(self, msg):
    """Robot pose callback
    """
    self.current_pose = msg


  def robot_state_cb(self, msg):
    """Robot state callback which each fsm will publish
    """
    with self.mutex:
      if msg.robot_id not in self.robots.keys():
        # New robot is joined
        color_vars = [[0, 0, 255], [255, 0, 0]]
        self.robots[msg.robot_id] = Robot(msg.robot_id, msg.pose, 1.0, color_vars[1], 0.5)
        self.robots[msg.robot_id].action = True # Robot action is occurred
      else:
        if self.current_pose:
          self.robots[msg.robot_id].update(msg)


  def start_cb(self, msg):
    """Start request callback
    """
    with self.mutex:
      self.start_request = msg.data
      self.logger.warning("Start requested: %d"%(self.start_request))


  def stop_cb(self, msg):
    """Stop request callback
    """
    with self.mutex:
      self.stop_request = msg.data
      self.logger.warning("Stop requested: %d"%(self.stop_request))


  def check_robot_action(self):
    """Check whether join/leave actions are occurred
    """
    with self.mutex:
      robot_action = False
      for robot_id, robot in self.robots.items():
        if robot.action:
          self.logger.warning("Action detected for robot: %d"%(robot_id))
          robot_action = True
          robot.action = False
    return robot_action

  
  def publish_robot_state(self):
    """Publish robot state
    """
    if not self.current_pose:
      return
    self.robot_state.header.stamp = rospy.Time.now()
    self.robot_state.header.frame_id = "map"
    self.robot_state.robot_id = self.robot.robot_id
    self.robot_state.pose = self.current_pose
    self.robot_state.working_state = self.state.value
    self.robot_state.traversed_cells = []
    self.state_pub.publish(self.robot_state)
        

  def active_cb(self):
    """Action server active callback
    """
    self.logger.warning("Action server is processing the goal")
    if self.stop_request:
      with self.mutex:
        self.stop_requested()
        self.stop_request = False
        self.action_client.cancel_goal()
        return
    if self.check_robot_action():
      self.robot_action()
      self.action_client.cancel_goal()
      return


  def done_cb(self, state, result):
    """Action server done callback
    """
    self.logger.warning("Task is finished")
    pass 


  def feedback_cb(self, feedback):
    """Action server feedback callback
    """
    self.logger.warning("Action server Feedback")
    if self.stop_request:
      with self.mutex:
        self.stop_requested()
        self.stop_request = False
        self.action_client.cancel_goal()
        return
    if self.check_robot_action():
      self.robot_action()
      self.action_client.cancel_goal()
      return
    self.publish_robot_state()

  
  def on_enter_IDLE(self):
    """IDLE state handler
    """
    loop_rate = rospy.Rate(1)
    while not rospy.is_shutdown():
      if self.start_request:
        with self.mutex:
          self.start_requested()
          self.start_request = False
          return
      elif self.stop_request:
        with self.mutex:
          self.stop_requested()
          self.stop_request = False
          return
      self.publish_robot_state()
      loop_rate.sleep()


  def on_enter_WORKING(self):
    """WORKING state handler
    """
    goal = PathFollowGoal()
    goal.path = self.cpp_path
    self.action_client.send_goal(goal, active_cb=self.active_cb, done_cb=self.done_cb, feedback_cb=self.feedback_cb)
    self.action_client.wait_for_result()

    result = self.action_client.get_result()


  def on_enter_CPP(self):
    """CPP state handler
    """
    self.logger.warning("Trying to compute robot regions..")
    self.publish_robot_state()
    voronoi = Voronoi(self.map_service, self.robots)
    for i in range(10):
      occ_grid, tesselation_image = voronoi.tesselation()

    occ_grid = cv2.resize(occ_grid, (self.graph.map_info.width, self.graph.map_info.height))
    tesselation_image = cv2.resize(tesselation_image, (self.graph.map_info.width, self.graph.map_info.height))

    map_info = self.graph.map_info
    path_plan = PathPlan(occ_grid, map_info, tesselation_image, self.robot)
    self.cpp_path = path_plan.plan()
    self.path_pub.publish(self.cpp_path)
    self.plan_created()


  def on_enter_STOP(self):
    """STOP state handler
    """
    self.publish_robot_state()
    exit()




#####################
###      MAIN     ###
#####################

if __name__ == "__main__":
  rospy.init_node("fsm")

  logging.getLogger("transitions").setLevel(logging.INFO)

  transitions = [['start_requested', States.IDLE, States.CPP],
                ['plan_created', States.CPP, States.WORKING],
                ['robot_action', States.WORKING, States.CPP],
                ['task_finished', States.WORKING, States.IDLE],
                ['stop_requested', '*', States.STOP],
                ['error', '*', States.STOP]]


  robot_name = rospy.get_param("~robot_name", "robot1")
  robot_id = rospy.get_param("~robot_id", 1)
  robot_size = rospy.get_param("~robot_size", 0.5)

  mcpp = MCPP(robot_name, robot_id, robot_size)
  machine = Machine(model=mcpp, states=States, transitions=transitions, initial=States.IDLE, queued=True)

  mcpp.to_IDLE()