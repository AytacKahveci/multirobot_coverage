from geometry_msgs.msg import Pose


class Node:
  """Node class
  """

  def __init__(self, row, col, pose):
    """Constructor
    """
    # Index of the node, type: int
    self.row = row
    self.col = col
    # Pose of the node, type: Pose()
    self.pose = pose
    # Neighbors of the node
    self.neighbors = []
    self.obstacle_neighbors = []
    # Robot id which is assigned to that node
    self.robot_id = -1
    # Cost for modified dijkstra algorithm
    self.cost = float('inf')
    self.power_dist = float('inf')
    # True if node has no obstacle
    self.valid = False
    # Root node for that node, type: Node
    self.s = None
    # Visited flag used for path planning
    self.visited = False
    self.belongs = True


  def __lt__(self, n):
    """Predicator used in priority queue
    
    Compares power dists of the nodes

    Args:
      n (Node): Node which is compared with

    Returns:
      bool: True if this node's power dist is lower than n's
    """
    return self.power_dist < n.power_dist

  
  def __repr__(self):
    return "Node[%d,%d] - robot_id:%d"%(self.row, self.col, self.robot_id)

  
  def get_index(self):
    """Get index of the node

    Returns:
      tuple(int, int): Node index (row, col)
    """
    return (self.row, self.col)

  
  def clear(self):
    """Clear assigned node properties
    """
    self.cost = float('inf')
    self.power_dist = float('inf')
    self.robot_id = -1
    self.valid = False
    self.visited = False

  
  def set_pose(self, pose):
    """Set pose of the node

    Args:
      pose (Pose): Pose which is assigned to
    """
    self.pose = pose

  
  def is_neighbor(self, n):
    """Check if node has given neighbor

    Args: 
      n (Node): Node which is searched in the this node's neighbors
    
    Return:
      bool: True if this node has neighbor (n), False otherwise

    Raise:
      ValueError: If neighbors list not constructed before this method call performed
    """
    if not self.neighbors:
      raise ValueError("Neighbor not initialized for node %d-%d"%(self.row, self.col))
    else:
      return n in self.neighbors