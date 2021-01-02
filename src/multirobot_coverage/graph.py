import os
import sys
import math
import cv2
from copy import deepcopy
import numpy as np

import rospy
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

from node import Node


class Graph:
  """Graph class
  """

  def __init__(self, map_service, resize=100):
    """Constructor

    Args:
      map_service (str): Map service name
    """
    # Occupancy grid map
    self.grid_map = None
    # Graph size
    self.height = 0
    self.width = 0
    self.resolution = 0
    self.resize = resize
    self.origin = Pose()
    # Nodes of the graph
    self.nodes = None
    self.occ_grid = None
    self.map_srv = rospy.ServiceProxy(map_service, GetMap)
    self.map_info = None
    resp = self.map_srv.call()
    
    # Create Graph
    self.create_graph(resp.map)


  def create_graph(self, map_msg):
    """Create graph from occupancy grid message

    Args:
      map_msg (OccupancyGrid): OccupancyGrid map
    """
    self.height = self.resize
    self.width = self.resize
    self.resolution_x = map_msg.info.resolution *  map_msg.info.width / self.resize
    self.resolution_y = map_msg.info.resolution *  map_msg.info.height / self.resize 
    self.resolution = map_msg.info.resolution *  map_msg.info.height * map_msg.info.width / (self.resize * self.resize)
    self.origin = map_msg.info.origin
    self.map_info = map_msg.info
    self.occ_grid = np.array(map_msg.data, dtype=np.uint8).reshape((map_msg.info.height, map_msg.info.width))
    self.occ_grid = self.occ_grid[::-1, :]
    self.occ_grid = cv2.resize(self.occ_grid, (self.resize, self.resize))


    self.nodes = np.empty((self.height, self.width), dtype=object)

    for i in range(self.height):
      for j in range(self.width):
        p = Pose()
        p.position.x = self.resolution_x * j + self.resolution_x / 2.0
        p.position.y = self.height * self.resolution_y - (self.resolution_y * i + self.resolution_y / 2.0)
        node = Node(i, j, p)
        if 0 <= self.occ_grid[i,j] <= 20:
          node.valid = True
        else:
          node.valid = False
        self.nodes[i,j] = node

    for i in range(self.height):
      for j in range(self.width):
        if self.nodes[i,j].valid:
          min_i = max(0, i-1)
          max_i = min(self.height - 1, i+1) + 1
          min_j = max(0, j-1)
          max_j = min(self.width - 1, j+1) + 1

          node = self.nodes[i,j]
          neighbors = self.nodes[min_i:max_i, min_j:max_j].flatten()
          for n in neighbors:
            if n != node:
              if n.valid:
                self.nodes[i,j].neighbors.append(n)
              else:
                self.nodes[i,j].obstacle_neighbors.append(n)
    """ for i in range(self.height):
      for j in range(self.width):
        node = self.nodes[i,j]
        print("****")
        print(node)
        print("Neighbors: ", )
        for n in node.neighbors:
          print("\t {}".format(n))
        print("######")
      if i > 7: 
          exit(0)
    exit(0) """


  def get_node(self, pose):
    """Get node from pose

    Args:
      pose (Pose): Requested pose

    Returns:
      Node: Node at the given pose if it is available, None otherwise
    """
    i, j = self.pose_to_index(pose)
    
    if 0 <= i < self.height and 0 <= j < self.width:
      #print("Found i:%d j:%d"%(i,j))
      return self.nodes[i,j]
    else:
      #print("Not Found i:%d j:%d"%(i,j))
      return None

  def get_node_from_index(self, i, j):
    """Get node from pose

    Args:
      pose (Pose): Requested pose

    Returns:
      Node: Node at the given pose if it is available, None otherwise
    """
    return self.nodes[i,j]

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
    height = self.height * self.resolution_y
    y = height - y

    i = int(math.floor(y / self.resolution_y))
    j = int(math.floor(x / self.resolution_x))
    return (i, j)