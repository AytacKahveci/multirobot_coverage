import math
import rospy
import numpy as np
from copy import deepcopy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
from robot import Robot
from node import Node


class Graph:
  def __init__(self, occ_grid, map_info, tesselation_image, robot):
    self.occ_grid = occ_grid
    self.height, self.width = occ_grid.shape
    self.resolution = map_info.resolution
    self.origin = map_info.origin
    self.tesselation_image = tesselation_image
    self.robot = robot
    """ self.height = self.height / (self.robot.size / self.resolution)
    self.width = self.width / (self.robot.size / self.resolution) """
    self.resized_height = int(math.ceil(self.height / math.ceil(self.robot.size / self.resolution)))
    self.resized_width = int(math.ceil(self.width / math.ceil(self.robot.size / self.resolution)))
    print("Height: ", self.height)
    print("Resized h: ", self.resized_height)
    self.nodes = np.empty((self.resized_height, self.resized_width), dtype=object)
    
    # Create graph
    self.create_graph()


  def create_graph(self):
    """Create graph from occupancy grid
    """
    robot_pix = int(math.ceil(self.robot.size / self.resolution))
    ii = 0
    jj = 0
    for i in range(0, self.height, robot_pix):
      jj = 0
      for j in range(0, self.width, robot_pix):
        block = self.occ_grid[i:i+robot_pix, j:j+robot_pix].flatten()
        avg = np.mean(block)
        robot_block = self.tesselation_image[i:i+robot_pix, j:j+robot_pix].flatten()
        n_occur = np.bincount(robot_block)
        block_id = np.argmax(n_occur)
        
        p = Pose()
        p.position.x = self.resolution * j + self.resolution / 2.0 + self.origin.position.x
        p.position.y = self.height * self.resolution - (self.resolution * i + self.resolution / 2.0) + self.origin.position.y
        node = Node(ii, jj, p)
        idx = np.where(block > 20)
        if block_id == self.robot.robot_id:
          if 0 <= avg <= 20:
            print("Node in path", node)
            node.valid = True
          else:
            node.valid = False
        elif block_id == 0:
          node.valid = False
        else:
          node.belongs = False
        self.nodes[ii,jj] = node
        jj += 1
      ii += 1


    height, width = self.nodes.shape
    print("Node shape: ", self.nodes.shape)
    for i in range(height):
      for j in range(width):
        min_i = max(0, i-1)
        max_i = min(height - 1, i+1) + 1
        min_j = max(0, j-1)
        max_j = min(width - 1, j+1) + 1

        node = self.nodes[i,j]
        neighbors = self.nodes[min_i:max_i, min_j:max_j].flatten()
        for n in neighbors:
          if not n or not node:
            print("None %d-%d"%(i,j))
            continue
          if n != node:
            if n.valid:
              print("Neighbor appended")
              self.nodes[i,j].neighbors.append(n)
            else:
              self.nodes[i,j].obstacle_neighbors.append(n)
    print("Graph is created!")


  def get_node(self, pose):
    """Get node from pose

    Args:
      pose (Pose): Requested pose

    Returns:
      Node: Node at the given pose if it is available, None otherwise
    """
    i, j = self.pose_to_index(pose)
    
    if 0 <= i < self.resized_height and 0 <= j < self.resized_width:
      print("Found i:%d j:%d"%(i,j))
      return self.nodes[i,j]
    else:
      print("Pose",pose)
      print("Not Found i:%d j:%d"%(i,j))
      return None


  def pose_to_index(self, pose):
    """Get index from pose

    Args:
      pose (Pose): Requested pose

    Returns:
      tuple(int, int): Index
    """
    x = pose.position.x
    y = pose.position.y
    x = x - self.origin.position.x
    y = y - self.origin.position.y
    print("Y: ", y)
    height = self.resized_height * self.robot.size
    print("Height: ", height)
    print("Resized height: ", self.resized_height)
    print("Robot size: ", self.robot.size)
    y = height - y

    i = int(math.floor(y / self.robot.size))
    j = int(math.floor(x / self.robot.size))
    return (i, j)


  def get_node_from_index(self, i, j):
    """Get node from index

    Args:
      i (int): Requested row index
      j (int): Requested col index

    Returns:
      Node: Node at the given index if it is available, None otherwise
    """
    if 0 <= i < self.resized_height and 0 <= j < self.resized_width:
      return self.nodes[i, j]
    else:
      return None


  def is_all_visited(self):
    """Check if all nodes in the graph are visited

    Returns:
      bool: True if all nodes are visited, False otherwise
    """
    cond = [node.visited if node and node.belongs and node.valid else True for node in self.nodes.flatten()]
    return all(cond)




class PathPlan:
  """Coverage path planning
  """
  def __init__(self, occ_grid, map_info, tesselation_image, robot):
    self.graph = Graph(occ_grid, map_info, tesselation_image, robot)
    self.robot = robot
    self.resulted_path = Path()
    self.resulted_path.header.frame_id = "map"
    robot_node = self.robot.get_node()
    self.start_node = self.graph.get_node(robot_node.pose)
    print("Start node is retreived")
  
  def add_to_path(self, node):
    pose = node.pose
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose = pose
    self.resulted_path.poses.append(p)


  def boustrophedon_motion(self, node):
    next_node = node
    next_node.visited = True
    max_i = self.graph.resized_height
    max_j = self.graph.resized_width

    path = np.zeros((max_i, max_j), dtype=np.int64)
    M = [next_node]

    num_iteration = 0
    while not rospy.is_shutdown():
      num_iteration += 1
      i, j = next_node.get_index()
      # North
      if i > 0:
        print("North")
        next_idx = (i-1, j)
        next_node = self.graph.get_node_from_index(*next_idx)
        if next_node:
          if next_node.valid and next_node not in M and not next_node.visited and next_node.belongs:
            M.append(next_node)
            self.add_to_path(next_node)
            path[next_idx] = num_iteration
            next_node.visited = True
            continue
      # South
      if i < (max_i - 1):
        print("South")
        next_idx = (i+1, j)
        next_node = self.graph.get_node_from_index(*next_idx)
        if next_node:
          if next_node.valid and next_node not in M and not next_node.visited and next_node.belongs:
            M.append(next_node)
            self.add_to_path(next_node)
            path[next_idx] = num_iteration
            next_node.visited = True
            continue
      # East
      if j < (max_j - 1):
        print("East")
        next_idx = (i, j+1)
        next_node = self.graph.get_node_from_index(*next_idx)
        if next_node:
          if next_node.valid and next_node not in M and not next_node.visited and next_node.belongs:
            M.append(next_node)
            self.add_to_path(next_node)
            path[next_idx] = num_iteration
            next_node.visited = True
            continue
      # West
      if j > 0:
        print("West")
        next_idx = (i, j-1)
        next_node = self.graph.get_node_from_index(*next_idx)
        if next_node:
          if next_node.valid and next_node not in M and not next_node.visited and next_node.belongs:
            M.append(next_node)
            self.add_to_path(next_node)
            path[next_idx] = num_iteration
            next_node.visited = True
            continue
      
      return path, M


  def backtrackin_f(self, node):
    max_i = self.graph.resized_height
    max_j = self.graph.resized_width
    i, j = node.get_index()
    neighbors = [(i, j+1), (i-1, j), (i, j-1), (i+1, j)]
    s = [None] * 4
    for i, neighbor in enumerate(neighbors):
      ii, jj = neighbor
      if 0 <= ii <= (max_i-1) and 0 <= jj <= (max_j-1):
        s[i] = self.graph.get_node_from_index(ii, jj)
          
    def b(si):
      if si == None:
        return 0
      
      if (si.valid and not si.visited):
        return 1
      else:
        return 0

    total = b(s[0]) + b(s[1]) + b(s[2]) + b(s[3])
    return total
  

  def calc_backtrack(self, M):
    back_l = []
    for node in M:
      if self.backtrackin_f(node) >= 1:
        back_l.append(node)
    
    def distance(n1, n2):
      i, j = n1.get_index()
      ii, jj = n2.get_index()
      return math.hypot((ii - i), (jj - j))

    scp = M[-1]
    print("Scp:",scp)
    print("back_l")
    for node in back_l:
      print(node)
    dist = [distance(scp, node) for node in back_l]
    if not dist:
      print("Not dist Scp:")
      return None
    idx = dist.index(min(dist))

    print("Scp pose:")
    print(back_l[idx].pose)
    return back_l[idx]


  def print_nodes(self):
    nodes = self.graph.nodes
    height, width = nodes.shape
    for i in range(height):
      s = ""
      for j in range(width):
        if not nodes[i, j]:
          print("(%d, %d) None"%(i,j))
          continue
        if not nodes[i,j].belongs:
          d = 2
        elif not nodes[i,j].valid:
          d = 3
        else:
          d = int(nodes[i,j].visited)
        s = s + str(d) + ","
      print(s + "EE" + str(i))


  def plan(self):
    node = self.start_node

    print("Path planning is started!")
    iteration = 0
    M_L = []
    while not rospy.is_shutdown():
      iteration += 1
      print("Iteration: %d"%iteration)
      path, M = self.boustrophedon_motion(node)
      M_L += M
      print("Last Node: ", M[-1])
      #self.print_nodes()
      """ if iteration == 6:
        return self.resulted_path """

      if self.graph.is_all_visited():
        print("Path planning is completed...")
        return self.resulted_path
      else:
        node = self.calc_backtrack(M)
        if not node:
          node = self.calc_backtrack(M_L)
        if not node:
          return self.resulted_path
        print("Next node", node)