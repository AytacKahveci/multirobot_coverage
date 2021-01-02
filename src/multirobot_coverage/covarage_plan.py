import cv2
from copy import copy
import numpy as np
from queue import PriorityQueue

from robot import Robot
from graph import Graph


class Voronoi:
  """Voronoi class
  """

  def __init__(self, map_service, robots):
    """Constructor

    Args:
      map_service (str): Map service name
      robots (dict(Robot)): Dict of robot objects
    """
    self.map_service = map_service
    self.robots = robots
    
    self.graph = Graph(map_service)
    self.rows = self.graph.height
    self.cols = self.graph.width
    self.tesselation_image = np.zeros((self.rows, self.cols, 3), dtype=np.uint8)
    self.tesselation_res = np.zeros((self.rows, self.cols), dtype=np.uint8)
    self.priority_queue = PriorityQueue()

    for robot in self.robots.values():
      robot.set_node(self.graph.get_node(robot.get_pose()))
      self.mark_robot(robot)


  def mark_node(self, node, robot):
    """Mark the node

    Args:
      node (Node): Node to be marked
      robot (Robot): Robot which marks the node
    """
    row, col = node.get_index()
    #print(row, col)
    for r in self.robots.values():
      self.tesselation_image[row, col, :] = [0, 0, 0]
      
    self.tesselation_image[row, col, :] = robot.color
    self.tesselation_res[row, col] = robot.robot_id
    robot.mass += 1

  
  @staticmethod
  def power_dist(x, r):
    """Calculate power dist

    Args:
      x (float): Node cost
      r (float): Robot weight

    Returns:
      float: Power dist
    """
    return pow(x, 2) - np.sign(r)*pow(r, 2)


  def get_robot(self, robot_id):
    """Get robot with given robot id

    Args:
      robot_id (int): Id of the robot

    Returns:
      Robot: Robot with given robot id if it exists, None otherwise
    """
    for robot in self.robots:
      if robot.robot_id == robot_id:
        return robot
    return None


  def mark_robot(self, robot):
    """Mark the node

    Args:
      node (Node): Node to be marked
      robot (Robot): Robot which marks the node
    """
    """ pose = robot.get_pose()
    node = self.graph.get_node(pose) """
    node = robot.get_node()
    row, col = node.get_index()
    min_i = max(0, row-20)
    max_i = min(self.graph.height - 1, row+20) + 1
    min_j = max(0, col-20)
    max_j = min(self.graph.width - 1, col+20) + 1
    #print(row, col)
    self.tesselation_image[min_i:max_i, min_j:max_j, 0] = 50


  def subtract(self, p1, p2):
    pn1 = np.array([p1.position.x, p1.position.y])
    pn2 = np.array([p2.position.x, p2.position.y])
    return np.subtract(pn1, pn2)


  def tesselation(self):
    """Tesselation
    """
    for robot in self.robots.values():
      self.mark_robot(robot)
      node = self.graph.get_node_from_index(*robot.get_node().get_index())
      print(node)
      node.cost = 0.0
      node.power_dist = node.cost - pow(robot.weight, 2)
      self.priority_queue.put((node.power_dist, node, robot.robot_id))
      self.mark_node(node, robot)
      for q in node.neighbors:
        if q is not node and not bool(set(q.obstacle_neighbors) and set(node.obstacle_neighbors)):
          q.s = q
    iterations = 0

    while not self.priority_queue.empty():
      #print("Iteration: ", iterations)
      iterations += 1
      elem = self.priority_queue.get()
      q = elem[1]
      if q.power_dist == float('inf'):
        #print("Inf break")
        break
      if q.robot_id is not -1:
        #print("Robot id continue")
        continue

      q.robot_id = elem[2]
      robot = self.robots[elem[2]]
      robot_node = self.graph.get_node_from_index(*robot.get_node().get_index())
      self.mark_node(q, robot)

      if q.s is not None:
        i_cl = q.cost * self.subtract(q.s.pose, robot.get_node().pose)
        robot.i_cl += i_cl

      for n in q.neighbors:
        _cost = q.cost + np.linalg.norm(self.subtract(q.pose, n.pose))
        _power_dist = self.power_dist(_cost, robot.weight)

        if _power_dist < n.power_dist:
          n.cost = _cost
          n.power_dist = _power_dist
          if not n.is_neighbor(robot_node):
            n.s = q.s
          self.priority_queue.put((n.power_dist, n, robot.robot_id))
        else:
          if n.robot_id is not -1:
            robot.neighbors[n.robot_id] = n.robot_id
            self.robots[n.robot_id].neighbors[robot.robot_id] = robot

    # Move robot to next pose for next iteration
    for robot in self.robots.values():
      print("Robot_prev node: ", robot.get_node())
      control_integral = robot.i_cl
      robot_node = self.graph.get_node_from_index(*robot.get_node().get_index())
      best_node = self.get_best_aligned_node(control_integral, robot_node)
      if best_node is None:
        print("Best node is none robot: %d", robot.robot_id)
        node_l = self.get_largest_weight_neighbor(robot_node)
        if node_l is not None:
          robot.set_node(node_l)
          print("Assigned pose: ", node_l.pose)
        else:
          raise RuntimeError("Largest weight neighbor is none")
      else:
        print("Next node: {}".format(best_node))
        robot.set_node(best_node)
        print("Assigned pose: ", best_node.pose)

    """ print("Tesselation image: ")
    for i in range(self.rows):
      row = ""
      for j in range(self.cols):
        row += str(self.tesselation_image[i,j]) + " "
      print(row) """
    cv2.imshow("src", self.tesselation_image[:,:,:])
    cv2.waitKey(50)

    self.clear()
    return self.graph.occ_grid, self.tesselation_res


  def clear(self):
    """Clear internal variables for next iterations
    """
    self.graph = Graph(self.map_service)
    for robot in self.robots.values():
      robot.clear()


  def get_best_aligned_node(self, i_func, robot_node):
    """Get best aligned node

    Args:
      i_func (np.ndarray): Integral
      robot_node (Node): Robot's current node

    Returns:
      Node: Best aligned node for the next iteration
    """
    max_dpi = 0
    best_node = None
    for n in robot_node.neighbors:
      if n != n.s:
        continue
      dpi = (np.dot(i_func, (self.subtract(n.pose, robot_node.pose))))
      print("Dpi: ", dpi)
      if dpi > max_dpi:
        max_dpi = dpi
        best_node = n
    return best_node


  def get_largest_weight_neighbor(self, robot_node):
    """Get neighbor node with the largest power dist

    Args:
      robot_node (Node): Robot's current node

    Returns:
      Node: Node with largest weight
    """
    cost = float("-inf")
    ans = None
    for n in robot_node.neighbors:
      if n.power_dist > cost:
        cost = n.power_dist
        ans = n
    return ans