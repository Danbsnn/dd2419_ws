#!/usr/bin/env python3
import math
import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from tf_transformations import quaternion_from_euler
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from shapely.geometry import Point as ShapePoint, Polygon


class GridPublisher(Node):
    def __init__(self):
        super().__init__('grid_publisher')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/map_objects', 10)
        self.odom_pose_pub = self.create_publisher(Pose, '/initial_pose', 10)

        self.create_subscription(PoseStamped, '/detected_object', self.detection_callback, 10)
        self.create_subscription(PoseStamped, '/box_detected', self.box_callback, 10)
        self.create_subscription(PointCloud2, '/lidar_map', self.pc_callback, 10)

        self.duplicate_threshold = 0.20
        self.pointcloud_update_period = 3.0

        self.latest_pc_msg = None
        self.latest_obstacle_mask = None

        self.og_timer = self.create_timer(0.2, self.publish_map)
        self.pc_timer = self.create_timer(self.pointcloud_update_period, self.update_pointcloud_mask)

        workspace_path = '/home/snowwhite/dd2419_ws/src/mapping/map/workspace_1.csv'
        map_path = '/home/snowwhite/dd2419_ws/src/mapping/map/map_1_1.csv'

        self.workspace = np.loadtxt(workspace_path, delimiter=',', skiprows=1) * 0.01
        self.origin_x = float(np.min(self.workspace[:, 0]))
        self.origin_y = float(np.min(self.workspace[:, 1]))
        self.workspace_poly = Polygon(self.workspace)

        raw_map = np.genfromtxt(
            map_path, delimiter=',', skip_header=1, dtype=None, encoding='utf-8'
        )
        self.object_types = [row[0] for row in raw_map]
        self.object_coords = [[row[1] * 0.01, row[2] * 0.01, row[3]] for row in raw_map]

        self.resolution = 0.05
        max_x = float(np.max(self.workspace[:, 0]))
        max_y = float(np.max(self.workspace[:, 1]))
        self.width = int(max_x / self.resolution)
        self.height = int(max_y / self.resolution)
        self.get_logger().info(f'Initialized {self.width}x{self.height} cells')

        self.start_x = None
        self.start_y = None
        for i, obj_type in enumerate(self.object_types):
            if obj_type == 'S':
                self.start_x = self.object_coords[i][0]
                self.start_y = self.object_coords[i][1]
                break

        if self.start_x is None:
            self.get_logger().error("No starting point 'S' found in map file!")
        else:
            init_pose_msg = Pose()
            init_pose_msg.position.x = float(self.start_x)
            init_pose_msg.position.y = float(self.start_y)
            init_pose_msg.position.z = 0.0
            init_pose_msg.orientation.w = 1.0
            self.odom_pose_pub.publish(init_pose_msg)
            self.get_logger().info(
                f'Published map→odom at ({self.start_x:.2f}, {self.start_y:.2f})'
            )

        self.og_grid = self.generate_workspace()
        self.dynamic_grid = self.og_grid.copy()
        self.latest_obstacle_mask = np.zeros((self.height, self.width), dtype=bool)

        rows, cols = np.indices((self.height, self.width))
        self.cell_x = cols * self.resolution + self.origin_x
        self.cell_y = rows * self.resolution + self.origin_y

        self.publish_objects()

    def pc_callback(self, msg: PointCloud2):
        self.latest_pc_msg = msg

    def update_pointcloud_mask(self):
        if self.latest_pc_msg is None:
            return

        points = np.asarray(
            list(point_cloud2.read_points(self.latest_pc_msg, field_names=('x', 'y', 'z'), skip_nans=True)),
            dtype=np.float32
        )
        if points.size == 0:
            self.latest_obstacle_mask = np.zeros((self.height, self.width), dtype=bool)
            return

        pts = points[:, :2]
        valid = (
            np.isfinite(pts[:, 0]) &
            np.isfinite(pts[:, 1]) &
            (pts[:, 0] >= self.origin_x) &
            (pts[:, 0] < self.origin_x + self.width * self.resolution) &
            (pts[:, 1] >= self.origin_y) &
            (pts[:, 1] < self.origin_y + self.height * self.resolution)
        )
        pts = pts[valid]
        if pts.shape[0] == 0:
            self.latest_obstacle_mask = np.zeros((self.height, self.width), dtype=bool)
            return

        gx = ((pts[:, 0] - self.origin_x) / self.resolution).astype(np.int32)
        gy = ((pts[:, 1] - self.origin_y) / self.resolution).astype(np.int32)

        in_bounds = (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)
        gx = gx[in_bounds]
        gy = gy[in_bounds]

        mask = np.zeros((self.height, self.width), dtype=bool)
        if gx.size > 0:
            mask[gy, gx] = True

        mask &= (self.og_grid != 100)
        self.latest_obstacle_mask = mask

        self.get_logger().info(
            f'Updated point-cloud mask with {int(np.count_nonzero(mask))} occupied cells'
        )

    def detection_callback(self, msg: PoseStamped):
        if msg.header.frame_id != 'map':
            return
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y
        for ox, oy, _ in self.object_coords:
            if math.hypot(new_x - ox, new_y - oy) < self.duplicate_threshold:
                return
        self.object_coords.append([new_x, new_y, 0])
        self.object_types.append('O')
        self.publish_objects()

    def box_callback(self, msg: PoseStamped):
        if msg.header.frame_id != 'map':
            return
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y
        for ox, oy, _ in self.object_coords:
            if math.hypot(new_x - ox, new_y - oy) < self.duplicate_threshold:
                return
        self.object_coords.append([new_x, new_y, 0])
        self.object_types.append('B')
        self.publish_objects()

    def publish_map(self):
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = self.resolution
        m.info.width = self.width
        m.info.height = self.height
        m.info.origin.position.x = self.origin_x
        m.info.origin.position.y = self.origin_y
        m.info.origin.position.z = 0.0

        grid = self.og_grid.copy()

        if self.latest_obstacle_mask is not None:
            grid[self.latest_obstacle_mask] = 100

        for i, (ox, oy, _) in enumerate(self.object_coords):
            if self.object_types[i] in ['O', 'B']:
                gx = int((ox - self.origin_x) / self.resolution)
                gy = int((oy - self.origin_y) / self.resolution)
                grid[max(0, gy - 1):gy + 2, max(0, gx - 1):gx + 2] = 100

        grid = self.update_visibility(grid)
        self.dynamic_grid = grid.copy()

        m.data = grid.flatten().tolist()
        self.map_pub.publish(m)
        self.publish_markers()

    def publish_markers(self):
        ma = MarkerArray()
        for i, (ox, oy, _) in enumerate(self.object_coords):
            obj_type = self.object_types[i]

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.action = Marker.ADD
            marker.pose.position.x = float(ox)
            marker.pose.position.y = float(oy)
            marker.pose.orientation.w = 1.0

            if obj_type == 'O':
                marker.type = Marker.CUBE
                marker.ns = 'O'
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.pose.position.z = 0.05
                marker.color.r = 1.0
                marker.color.a = 1.0
            elif obj_type == 'B':
                marker.type = Marker.CUBE
                marker.ns = 'B'
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.pose.position.z = 0.05
                marker.color.r = 0.8
                marker.color.g = 0.8
                marker.color.b = 0.8
                marker.color.a = 1.0
            elif obj_type == 'S':
                marker.type = Marker.SPHERE
                marker.ns = 'S'
                marker.scale.x = 0.15
                marker.scale.y = 0.1
                marker.scale.z = 0.15
                marker.color.b = 1.0
                marker.color.a = 0.8

            ma.markers.append(marker)

        self.marker_pub.publish(ma)

    def publish_objects(self):
        static_transforms = []
        for i, (ox, oy, angle) in enumerate(self.object_coords):
            if self.object_types[i] not in ['O', 'B']:
                continue

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = f"{'Box' if self.object_types[i] == 'B' else 'Cube'}_{i}"

            t.transform.translation.x = float(ox)
            t.transform.translation.y = float(oy)
            t.transform.translation.z = 0.0

            q = quaternion_from_euler(0.0, 0.0, angle)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            static_transforms.append(t)

        if static_transforms:
            self.tf_static_broadcaster.sendTransform(static_transforms)

    def generate_workspace(self):
        grid = np.full((self.height, self.width), 100, dtype=np.int8)

        for r in range(self.height):
            for c in range(self.width):
                x = c * self.resolution + self.origin_x
                y = r * self.resolution + self.origin_y
                if self.workspace_poly.contains(ShapePoint(x, y)):
                    grid[r, c] = -1

        return grid

    def update_visibility(self, grid):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return grid

        rx = transform.transform.translation.x
        ry = transform.transform.translation.y
        orientation = transform.transform.rotation

        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        cam_offset = 0.1
        length = 0.8
        width = 0.5

        camx = rx + cam_offset * math.cos(yaw)
        camy = ry + cam_offset * math.sin(yaw)

        dx = self.cell_x - camx
        dy = self.cell_y - camy
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy

        visible = (
            (grid == -1) &
            (forward > 0.0) &
            (forward < length) &
            (np.abs(lateral) < width / 2.0)
        )
        grid[visible] = 0
        return grid


def main():
    rclpy.init()
    node = GridPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
