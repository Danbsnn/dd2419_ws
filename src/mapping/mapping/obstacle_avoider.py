#!/usr/bin/env python3

import math
import heapq
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

from scipy.ndimage import maximum_filter


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obs_avo')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )

        self.reached_sub = self.create_subscription(
            Bool, '/target_reached', self.reached_callback, 10
        )

        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal', 10
        )

        self.path_pub = self.create_publisher(
            Path, '/planned_path', 10
        )

        self.map_msg = None
        self.grid = None  # 2D numpy occupancy grid

        self.goal_queue = []
        self.goal_index = 0

        self.occupied_threshold = 100

        self.get_logger().info("Path Planner running...")

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)

        occupied = self.grid >= self.occupied_threshold  # Unknown is unoccupied
        inflation_radius_m = 0.25
        r = int(inflation_radius_m / msg.info.resolution)
        inflated = maximum_filter(occupied.astype(np.uint8), size=2*r+1) > 0
        self.free_mask = ~inflated

    def goal_callback(self, goal_msg: PoseStamped):
        if self.map_msg is None or self.grid is None:
            self.get_logger().warn("Waiting for map.")
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        start_x, start_y, _ = robot_pose
        goal_x = goal_msg.pose.position.x
        goal_y = goal_msg.pose.position.y

        start = self.world_to_grid(start_x, start_y)
        goal = self.world_to_grid(goal_x, goal_y)

        if not self.is_free_cell(start):
            self.get_logger().warn("Robot start cell is occupied.")
            return

        if not self.is_free_cell(goal):
            self.get_logger().warn("Goal cell is occupied.")
            return

        path_cells = self.theta_star(start, goal)
        if path_cells is None:
            self.get_logger().warn("No path found.")
            return

        waypoint_cells = self.sparsify_path(path_cells)
        # print(waypoint_cells)
        self.goal_list = [self.cell_to_pose(cell) for cell in waypoint_cells]
        self.goal_index = 0

        self.publish_path_from_cells(path_cells, goal_msg.pose.orientation)

        if self.goal_list:
            self.get_logger().info(f"Path found with {len(self.goal_list)} waypoint(s).")
            self.goal_pub.publish(self.goal_list[0])

    def reached_callback(self, msg: Bool):
        if not msg.data or not self.goal_list:
            return

        self.goal_index += 1

        if self.goal_index < len(self.goal_list):
            self.goal_pub.publish(self.goal_list[self.goal_index])
        else:
            self.get_logger().info("Finished planned path.")
            self.goal_list = []
            self.goal_index = 0

    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            return (x, y, yaw)

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None


    def world_to_grid(self, x, y):
        info = self.map_msg.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        info = self.map_msg.info
        x = info.origin.position.x + (gx + 0.5) * info.resolution
        y = info.origin.position.y + (gy + 0.5) * info.resolution
        return (x, y)

    def is_free_cell(self, cell):
        gx, gy = cell

        if gx < 0 or gy < 0 or gx >= self.map_msg.info.width or gy >= self.map_msg.info.height:
            return False

        return self.free_mask[gy, gx]

    def neighbors8(self, cell):
        x, y = cell

        for dx, dy in [
            (-1, -1), (0, -1), (1, -1),
            (-1,  0),          (1,  0),
            (-1,  1), (0,  1), (1,  1),
        ]:
            nx, ny = x + dx, y + dy

            if 0 <= nx < self.map_msg.info.width and 0 <= ny < self.map_msg.info.height:
                if self.free_mask[ny, nx]:
                    yield (nx, ny)

    def heuristic(self, a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def cost(self, a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def line_of_sight(self, a, b):
        x0, y0 = a
        x1, y1 = b

        n = max(abs(x1 - x0), abs(y1 - y0)) + 1
        xs = np.rint(np.linspace(x0, x1, n)).astype(np.int32)
        ys = np.rint(np.linspace(y0, y1, n)).astype(np.int32)

        if np.any(xs < 0) or np.any(xs >= self.map_msg.info.width):
            return False
        if np.any(ys < 0) or np.any(ys >= self.map_msg.info.height):
            return False

        return np.all(self.free_mask[ys, xs])

    def theta_star(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, (self.heuristic(start, goal), start))

        g = {start: 0.0}
        parent = {start: start}
        closed = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in closed:
                continue

            if current == goal:
                return self.reconstruct_path(parent, goal)

            closed.add(current)

            for nbr in self.neighbors8(current):
                if nbr in closed:
                    continue

                if nbr not in g:
                    g[nbr] = float('inf')
                    parent[nbr] = None

                p = parent[current]

                # Theta*: try connecting through current's parent
                if p is not None and self.line_of_sight(p, nbr):
                    new_g = g[p] + self.cost(p, nbr)
                    new_parent = p
                else:
                    new_g = g[current] + self.cost(current, nbr)
                    new_parent = current

                if new_g < g[nbr]:
                    g[nbr] = new_g
                    parent[nbr] = new_parent
                    f = new_g + self.heuristic(nbr, goal)
                    heapq.heappush(open_heap, (f, nbr))

        return None

    def reconstruct_path(self, parent, goal):
        path = [goal]
        cur = goal

        while parent[cur] != cur:
            cur = parent[cur]
            path.append(cur)

        path.reverse()
        return path

    def sparsify_path(self, path):
        result = []

        prev_dir = (
            path[1][0] - path[0][0],
            path[1][1] - path[0][1]
        )

        for i in range(1, len(path) - 1):
            new_dir = (
                path[i + 1][0] - path[i][0],
                path[i + 1][1] - path[i][1]
            )

            if new_dir != prev_dir:
                result.append(path[i])

            prev_dir = new_dir

        result.append(path[-1])

        return result

    def cell_to_pose(self, cell):
        x, y = self.grid_to_world(*cell)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        # pose.pose.orientation = orientation
        return pose

    def publish_path_from_cells(self, path_cells, orientation):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for cell in path_cells:
            x, y = self.grid_to_world(*cell)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation = orientation
            path.poses.append(pose)

        self.path_pub.publish(path)


def main():
    rclpy.init()
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
