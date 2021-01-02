import numpy as np
from enum import Enum
from multirobot_coverage.msg import RobotState
from geometry_msgs.msg import Pose


class States(Enum):
  """FSM states
  """
  IDLE = 0
  CPP = 1
  WORKING = 2
  STOP = 3


class Robot:
  """Robot class
  """

  def __init__(self, robot_id, pose, weight, color, size = 0.5):
    """Constructor

    Args:
      robot_id (int): Robot id
      pose (Pose): Robot pose
      weight (float): Robot weight
      color (list(int, int, int)): RGB color of the robot
    """
    self.robot_id = robot_id
    self.pose = pose
    self.weight = weight
    self.color = color
    self.size = size
    self.node = None
    self.mass = 0
    self.i_cl = np.zeros((2,))
    # Neighbors of the robot
    self.neighbors = {}
    self.traversed_cells = []
    self.working_state = States.IDLE
    self.prev_state = States.IDLE
    self.action = False


  def get_pose(self):
    """Get robot pose

    Returns:
      Pose: Robot pose
    """
    return self.pose


  def get_node(self):
    """Get robot's node in the graph
    """
    return self.node


  def set_node(self, node):
    """Set robot node

    Args:
      node (Node): Robot node to be set
    """
    self.node = node


  def update(self, msg):
    """Update robot state

    Args:
      msg (Pose): Robot state to be updated
    """
    self.pose = msg.pose.pose
    self.traversed_cells = msg.traversed_cells
    self.working_state = States(msg.working_state)

    # If state has changed and robot works, inform that the robot sends join/leave action
    if self.prev_state in [States.IDLE, States.STOP] and self.working_state in [States.WORKING, States.CPP]:
      self.action = True
      self.prev_state = self.working_state
    elif self.prev_state in [States.WORKING, States.CPP] and self.working_state in [States.IDLE, States.STOP]:
      self.action = True 
      self.prev_state = self.working_state


  def clear(self):
    """Clear robot states
    """
    self.neighbors = {}
    self.mass = 0
    self.i_cl = np.zeros((2,))