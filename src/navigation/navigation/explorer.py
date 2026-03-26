#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from robp_interfaces.srv import GetNextFrontier
import math

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')

        # Initializing variables
        self.map = None
        self.robot_position = None
        
        # Publisher for goal
        # self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)

        # Subscriber for robot position
        self.loc_sub = self.create_subscription(
            PoseStamped,
            '/localized_pose',
            self.loc_callback,
            10
        )

        # Subscriber for occupancy map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # GetNextFrontier service
        self.frontier_srv = self.create_service(
            GetNextFrontier, 
            'get_next_frontier', 
            self.handle_frontier_request
        )

    # Subscribers callbacks
    def loc_callback(self, msg):
        self.robot_position = msg

    def map_callback(self, msg):
        self.map = msg

    # Service callback
    def handle_frontier_request(self, request, response):

        if self.map is None or self.robot_position is None:
            response.success = False
            return response
        
        frontiers = self.detect_frontiers()

        # Exploration done case
        if not frontiers:
            response.success = False
            return response
        
        centers = self.compute_frontier_centers(frontiers)
        best = self.choose_best_frontier(centers)

        response.success = True
        response.x = best[0]
        response.y = best[1]

        return response

    # Functions for frontier detection and selection
    def detect_frontiers(self):
        frontiers = []
        # find the differents boudaries cells of the map
        width = self.map.info.width
        height = self.map.info.height
        data = self.map.data

        for y in range(1, height-1):
            for x in range(1, width-1):
                idx = y * width + x # we create a unique index per cell
                val = data[idx]

                # ignore occupied cells and unknown cells 
                if val !=0:
                    continue
                
                # look at neighbors of the free cell
                neighbors = [
                    data[idx-1], 
                    data[idx+1], 
                    data[idx-width], 
                    data[idx+width]]

                # if any neighbor is unknown, this cell is a frontier
                if -1 in neighbors:
                    frontiers.append((x, y))

        return frontiers
    
    def compute_frontier_centers(self, frontiers):
        # Initialisation of variables
        visited = set() # use of set instead of list for faster research of visited cells
        clusters = []
        frontier_set = set(frontiers)

        # clusters creation with breadth first search algorithm
        for cell in frontiers:
            if cell in visited: # avoid computation of same cluster multiple times
                continue

            cluster = []
            queue = [cell]
            visited.add(cell)

            while queue:
                cx, cy = queue.pop(0)
                cluster.append((cx, cy))

                # neighbors of the cell in the cluster we are building
                neighbors = [
                    (cx+1, cy), (cx-1, cy),
                    (cx, cy+1), (cx, cy-1),
                    (cx+1, cy+1), (cx-1, cy-1),
                    (cx+1, cy-1), (cx-1, cy+1)
                ]

                # if neighbor is a frontier and is not in a cluster yet
                for nx, ny in neighbors:
                    if (nx, ny) in frontier_set and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        queue.append((nx, ny))

            clusters.append(cluster)

        # center of cluster computation
        centers = []
        for cluster in clusters:
            mean_x = sum(c[0] for c in cluster) / len(cluster)
            mean_y = sum(c[1] for c in cluster) / len(cluster)

            # take the cell of the cluster the closest to the mean of the cluster
            best_cell = min(cluster, key=lambda c: (c[0]-mean_x)**2 + (c[1]-mean_y)**2)

            centers.append(self.grid_to_world(best_cell[0], best_cell[1]))

        return centers

    # find cluster center closest to robot position
    def choose_best_frontier(self, centers):

        rx = self.robot_position.pose.position.x
        ry = self.robot_position.pose.position.y

        best = None
        best_dist = float("inf")

        for cx, cy in centers:
            d = math.hypot(cx - rx, cy - ry) # distance calculation

            if d < best_dist:
                best_dist = d
                best = (cx, cy)

        return best

    def grid_to_world(self, gx, gy):
        res = self.map.info.resolution
        origin = self.map.info.origin.position

        wx = origin.x + gx * res
        wy = origin.y + gy * res
        
        return wx, wy
        

def main():
    rclpy.init()
    node = Explorer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()