#!/usr/bin/env python3

import math
import heapq
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from robp_interfaces.msg import PoseStampedWithType
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

from scipy.ndimage import distance_transform_edt
from geometry_msgs.msg import PoseStamped


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        self.goal_sub = self.create_subscription(
            PoseStampedWithType, '/goal_pose', self.goal_callback, 10
        )

        self.reached_sub = self.create_subscription(
            Bool, '/target_reached', self.reached_callback, 10
        )

        self.path_pub = self.create_publisher(
            Path, '/planned_path', 10
        )

        self.goal_type = None
        self.map_msg = None
        self.free_mask = None
        self.grid = None  # 2D numpy occupancy grid
        self.cost_map = None
        self.cost_map_inflation = 0.6

        self.occupied_threshold = 100

        self.obst_cost = 3

        self.get_logger().info("Path Planner running...")

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)

        occupied = self.grid >= self.occupied_threshold  # Unknown is unoccupied
        self.free_mask = ~occupied

        # Distance from every free cell to the nearest occupied cell, in meters.
        dist_m = distance_transform_edt(self.free_mask) * msg.info.resolution
        self.cost_map = np.zeros_like(dist_m, dtype=np.float32)
        near_obstacle = (self.free_mask) & (dist_m < self.cost_map_inflation)
        self.cost_map[near_obstacle] = (
                (self.cost_map_inflation - dist_m[near_obstacle])
                / self.cost_map_inflation
            ) ** 2
        self.cost_map[occupied] = np.inf

    def goal_callback(self, goal_msg: PoseStampedWithType):
        if self.map_msg is None or self.grid is None:
            self.get_logger().warn("Waiting for map.")
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        start_x, start_y, _ = robot_pose
        goal_x = goal_msg.pose.pose.position.x
        goal_y = goal_msg.pose.pose.position.y
        self.goal_type = goal_msg.type

        start = self.world_to_grid(start_x, start_y)
        goal = self.world_to_grid(goal_x, goal_y)

        if not self.is_free_cell(start):
                    self.get_logger().warn("Robot start cell is occupied.")
                    return

        if self.goal_type not in ["B", "O"]:
            if not self.is_free_cell(goal):
                self.get_logger().warn("Goal cell is occupied.")
                return

        if self.goal_type in ["B"]:
            q = goal_msg.pose.pose.orientation
            (_, _, goal_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

            offset = 0.45
            # fx = math.cos(goal_yaw)
            # fy = math.sin(goal_yaw) 
            lx = -math.sin(goal_yaw)
            ly = math.cos(goal_yaw)

            # Candidates around the box
            candidate_positions = [
                #(goal_x + offset * fx, goal_y + offset * fy),  # front
                #(goal_x - offset * fx, goal_y - offset * fy),  # back
                (goal_x + offset * lx, goal_y + offset * ly, goal_x+0.1 * lx, goal_y+0.1 * ly),  # left
                (goal_x - offset * lx, goal_y - offset * ly, goal_x-0.1 * lx, goal_y-0.1 * ly),  # right
            ]
            for candidate in candidate_positions:
                if self.is_free_cell(self.world_to_grid(candidate[0], candidate[1])):
                    goal_candidate = candidate
                    break

            goal = self.world_to_grid(goal_candidate[0], goal_candidate[1])
            final_goal = self.world_to_grid(goal_candidate[2], goal_candidate[3])

            path_cells = self.a_star(start, goal)
            if path_cells is None:
                self.get_logger().warn("No path found.")
                return

            path_cells.append(final_goal)
            self.publish_path_from_cells(path_cells)
            return

        path_cells = self.a_star(start, goal)
        if path_cells is None:
            self.get_logger().warn("No path found.")
            return

        self.publish_path_from_cells(path_cells)

    def reached_callback(self, msg: Bool):
        self.get_logger().info(f"Target reached!")

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

    def reached_planner_goal(self, current, goal, goal_radius_cells):
        if goal_radius_cells <= 0:
            return current == goal

        dx = current[0] - goal[0]
        dy = current[1] - goal[1]

        return math.hypot(dx, dy) <= goal_radius_cells

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

    def in_bounds(self, cell):
        gx, gy = cell
        return (
            0 <= gx < self.map_msg.info.width
            and 0 <= gy < self.map_msg.info.height
        )

    def is_free_cell(self, cell):
        if not self.in_bounds(cell):
            return False
        gx, gy = cell
        return bool(self.free_mask[gy, gx])

    def neighbors8(self, cell):
        x, y = cell

        for dx, dy in [
            (-1, -1), (0, -1), (1, -1),
            (-1,  0),          (1,  0),
            (-1,  1), (0,  1), (1,  1),
        ]:
            nbr = (x + dx, y + dy)
            if self.is_free_cell(nbr):
                yield nbr

    def heuristic(self, a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def cost(self, a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def line_indices(self, a, b):
        x0, y0 = a
        x1, y1 = b

        n = max(abs(x1 - x0), abs(y1 - y0)) + 1
        xs = np.rint(np.linspace(x0, x1, n)).astype(np.int32)
        ys = np.rint(np.linspace(y0, y1, n)).astype(np.int32)
        return xs, ys

    def line_of_sight(self, a, b):
        xs, ys = self.line_indices(a, b)

        if np.any(xs < 0) or np.any(xs >= self.map_msg.info.width):
            return False
        if np.any(ys < 0) or np.any(ys >= self.map_msg.info.height):
            return False

        return bool(np.all(self.free_mask[ys, xs]))

    def path_cost(self, path):
        if path is None or len(path) < 2:
            return 0.0
        return sum(self.step_cost(path[i], path[i + 1]) for i in range(len(path) - 1))

    def step_cost(self, a, b):
        ax, ay = a
        bx, by = b

        base = math.hypot(bx - ax, by - ay)
        cell_cost = 0.5 * (
            float(self.cost_map[ay, ax]) + float(self.cost_map[by, bx])
        )
        return base * (1.0 + self.obst_cost * cell_cost)

    def a_star(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, (self.heuristic(start, goal), start))

        g = {start: 0.0}
        parent = {start: start}
        closed = set()

        if self.goal_type == "O":
            goal_radius_m = 0.16
            goal_radius_cells = int(goal_radius_m / self.map_msg.info.resolution)
        else:
            goal_radius_cells = 0

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in closed:
                continue

            if self.reached_planner_goal(current, goal, goal_radius_cells):
                return self.reconstruct_path(parent, current)

            closed.add(current)

            for nbr in self.neighbors8(current):
                if nbr in closed:
                    continue

                new_g = g[current] + self.step_cost(current, nbr)

                if new_g < g.get(nbr, float('inf')):
                    g[nbr] = new_g
                    parent[nbr] = current
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
        return pose

    def publish_path_from_cells(self, path_cells):
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
            path.poses.append(pose)

        self.path_pub.publish(path)


def main():
    rclpy.init()
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()