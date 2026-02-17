#!/usr/bin/env python

import rcply
from rcply.node import Node
import numpy as np
import heapq

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

from occupancy_grid_map.workspace_utils import Workspace




class AstarNode(Node):
    def __init__(self):
        super().__init__('astar_planning')

        self.ws = Workspace()
        self.grid = None
        self.goal = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseStamped, '/planner/next_goal', self.goal_callback, 10)


        def map_callback(self, msg):
            self.resolution = msg.info.resolution
            self.width = msg.info.width
            self.height = msg.info.height

            grid = np.array(msg.data, dtype=np.int8).reshape(
                (self.height, self.width)
            )

            # conver ROS format to A* format
            # free: 0 -> 0
            # accupied: 100 -> 1
            self.grid = np.where(grid == 100, 1, 0)

            self.grid_received = True

        def goal_callback(self, msg):
            self.goal = msg.pose.position
            

