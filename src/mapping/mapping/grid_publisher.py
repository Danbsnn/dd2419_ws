#!/usr/bin/env python
import time
import rclpy
import math
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from shapely.geometry import Point as ShapePoint, Polygon
from tf_transformations import quaternion_from_euler

from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from tf2_geometry_msgs import do_transform_pose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.duration import Duration

from robp_interfaces.msg import Object, ObjectList
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from scipy.ndimage import binary_dilation


class GridPublisher(Node):

    def __init__(self):

        super().__init__('grid_publisher')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/map_objects',
            10
        )

        self.odom_pose_pub = self.create_publisher(
            Pose,
            '/initial_pose',
            10
        )

        self.object_pub = self.create_publisher(
            ObjectList,
            '/object_list',
            qos
        )

        # ==================================================
        # Subscribers
        # ==================================================

        self.detection_sub = self.create_subscription(
            PoseStamped,
            '/detected_object',
            self.detection_callback,
            10
        )

        self.box_sub = self.create_subscription(
            PoseStamped,
            '/box_detected',
            self.box_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/odom_pose',
            self.pose_callback,
            10
        )

        self.create_subscription(
            PointCloud2,
            '/lidar_map',
            self.pc_callback,
            10
        )

        # ==================================================
        # Variables
        # ==================================================

        self.robot_pose = None

        self.detection_range = 0.9
        self.duplicate_threshold = 0.40

        self.latest_pc_msg = None
        self.latest_obstacle_mask = None

        # ==================================================
        # Timers
        # ==================================================

        self.og_timer = self.create_timer(
            0.2,
            self.publish_map
        )

        self.pc_timer = self.create_timer(
            3,
            self.update_pointcloud_mask
        )

        # ==================================================
        # Load workspace/map
        # ==================================================

        workspace_path = "/home/snowwhite/dd2419_ws/src/mapping/map/workspace_1.csv"
        map_path = "/home/snowwhite/dd2419_ws/src/mapping/map/map_1_1.csv"

        self.workspace = np.loadtxt(
            workspace_path,
            delimiter=',',
            skiprows=1
        ) * 0.01

        self.origin_x = np.min(self.workspace[:, 0])
        self.origin_y = np.min(self.workspace[:, 1])

        raw_map = np.genfromtxt(
            map_path,
            delimiter=',',
            skip_header=1,
            dtype=None,
            encoding='utf-8'
        )

        self.object_types = [row[0] for row in raw_map]

        self.object_coords = [
            [row[1] * 0.01, row[2] * 0.01, row[3]]
            for row in raw_map
        ]

        self.resolution = 0.05

        max_x = int(np.max(self.workspace[:, 0]))
        max_y = int(np.max(self.workspace[:, 1]))

        self.width = int(max_x / self.resolution)
        self.height = int(max_y / self.resolution)

        self.get_logger().info(
            f"Initialized {self.width}x{self.height} cells"
        )

        # ==================================================
        # Start pose
        # ==================================================

        self.start_x = None
        self.start_y = None

        for i, obj_type in enumerate(self.object_types):

            if obj_type == 'S':

                self.start_x = self.object_coords[i][0]
                self.start_y = self.object_coords[i][1]

                break

        if self.start_x is not None:

            init_pose_msg = Pose()

            init_pose_msg.position.x = float(self.start_x)
            init_pose_msg.position.y = float(self.start_y)

            init_pose_msg.orientation.w = 1.0

            self.odom_pose_pub.publish(init_pose_msg)

        # ==================================================
        # TF
        # ==================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ==================================================
        # Grids
        # ==================================================

        self.og_grid = self.generate_workspace()

        self.dynamic_grid = self.og_grid.copy()

        self.publish_objects()

    # ======================================================
    # Pointcloud
    # ======================================================

    def pc_callback(self, msg: PointCloud2):

        self.latest_pc_msg = msg

    def update_pointcloud_mask(self):

        if self.latest_pc_msg is None:
            return

        points = point_cloud2.read_points(
            self.latest_pc_msg,
            field_names=('x', 'y'),
            skip_nans=True
        )

        pts = np.array(
            [[p[0], p[1]] for p in points],
            dtype=np.float32
        )

        if pts.size == 0:

            self.latest_obstacle_mask = np.zeros(
                (self.height, self.width),
                dtype=bool
            )

            return

        valid = (
            np.isfinite(pts[:, 0]) &
            np.isfinite(pts[:, 1])
        )

        pts = pts[valid]

        gx = (
            (pts[:, 0] - self.origin_x) /
            self.resolution
        ).astype(np.int32)

        gy = (
            (pts[:, 1] - self.origin_y) /
            self.resolution
        ).astype(np.int32)

        in_bounds = (
            (gx >= 0) &
            (gx < self.width) &
            (gy >= 0) &
            (gy < self.height)
        )

        gx = gx[in_bounds]
        gy = gy[in_bounds]

        mask = np.zeros(
            (self.height, self.width),
            dtype=bool
        )

        if gx.size > 0:
            mask[gy, gx] = True

        mask &= (self.og_grid != 100)

        mask = binary_dilation(
            mask,
            structure=np.ones((3, 3), dtype=bool)
        )

        self.latest_obstacle_mask = mask

    # ======================================================
    # Cube callback
    # ======================================================

    def detection_callback(self, msg: PoseStamped):

        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )

            transformed_pose = PoseStamped()

            transformed_pose.header.frame_id = 'map'

            transformed_pose.pose = do_transform_pose(
                msg.pose,
                transform
            )

        except Exception as e:

            self.get_logger().warn(
                f"TF transform failed: {e}"
            )

            return

        new_x = transformed_pose.pose.position.x
        new_y = transformed_pose.pose.position.y

        valid = (
            (new_x >= self.origin_x) and
            (new_x < self.origin_x + self.width * self.resolution) and
            (new_y >= self.origin_y) and
            (new_y < self.origin_y + self.height * self.resolution)
        )

        if not valid:

            self.get_logger().info(
                "Object outside workspace perimeter"
            )

            return

        is_duplicate = False

        for i, (ox, oy, _) in enumerate(self.object_coords):
            if self.object_types[i] != "O":
                continue

            if math.hypot(
                new_x - ox,
                new_y - oy
            ) < self.duplicate_threshold:

                is_duplicate = True
                break

        if not is_duplicate:

            self.get_logger().info(
                f"New object discovered at "
                f"({new_x:.2f}, {new_y:.2f})"
            )

            self.object_coords.append(
                [new_x, new_y, 0]
            )

            self.object_types.append('O')

            self.sync_full_map_to_csv()

            self.publish_objects()

    # ======================================================
    # BOX CALLBACK (FIXED)
    # ======================================================

    def box_callback(self, msg: PoseStamped):

        if msg.header.frame_id != 'map':

            self.get_logger().warn(
                f"Detected box is in "
                f"'{msg.header.frame_id}' frame. "
                f"Please change to 'map'!"
            )

            return

        new_x = msg.pose.position.x
        new_y = msg.pose.position.y

        valid = (
            (new_x >= self.origin_x) and
            (new_x < self.origin_x + self.width * self.resolution) and
            (new_y >= self.origin_y) and
            (new_y < self.origin_y + self.height * self.resolution)
        )

        if not valid:

            self.get_logger().info(
                "Box outside workspace perimeter"
            )

            return

        is_duplicate = False

        for i, (ox, oy, _) in enumerate(self.object_coords):
            if self.object_types[i] != "B":
                continue
            distance = math.hypot(
                new_x - ox,
                new_y - oy
            )

            # reject near ANY object/cube/box
            if distance < 2*self.duplicate_threshold:

                is_duplicate = True

                self.get_logger().info(
                    f"Duplicate/near object detected "
                    f"({distance:.2f}m)"
                )

                break

        if not is_duplicate:

            self.get_logger().info(
                f"New box discovered at "
                f"({new_x:.2f}, {new_y:.2f})"
            )

            self.object_coords.append(
                [new_x, new_y, 0]
            )

            self.object_types.append('B')

            # IMPORTANT FIX
            self.sync_full_map_to_csv()

            self.publish_objects()

    # ======================================================
    # Pose callback
    # ======================================================

    def pose_callback(self, msg: PoseStamped):

        self.robot_pose = msg

    # ======================================================
    # Publish map
    # ======================================================

    def publish_map(self):

        m = OccupancyGrid()

        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()

        m.info.resolution = self.resolution
        m.info.width = self.width
        m.info.height = self.height

        m.info.origin.position.x = self.origin_x
        m.info.origin.position.y = self.origin_y

        grid = self.dynamic_grid

        if self.latest_obstacle_mask is not None:
            grid[self.latest_obstacle_mask] = 100

        for i, (ox, oy, _) in enumerate(self.object_coords):

            if self.object_types[i] in ['O', 'B']:

                gx = int(
                    (ox - self.origin_x) /
                    self.resolution
                )

                gy = int(
                    (oy - self.origin_y) /
                    self.resolution
                )

                grid[
                    max(0, gy - 1):gy + 2,
                    max(0, gx - 1):gx + 2
                ] = 100

        m.data = grid.flatten().tolist()

        self.map_pub.publish(m)

        self.publish_markers()

    # ======================================================
    # Markers
    # ======================================================

    def publish_markers(self):

        ma = MarkerArray()

        for i, (ox, oy, _) in enumerate(self.object_coords):

            obj_type = self.object_types[i]

            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.id = i

            marker.action = Marker.ADD

            marker.pose.position.x = float(ox)
            marker.pose.position.y = float(oy)

            marker.pose.orientation.w = 1.0

            if obj_type == 'O':

                marker.type = Marker.CUBE
                marker.ns = "O"

                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                marker.pose.position.z = 0.05

                marker.color.r = 1.0
                marker.color.a = 1.0

            elif obj_type == 'B':

                marker.type = Marker.CUBE
                marker.ns = "B"

                marker.scale.x = 0.24
                marker.scale.y = 0.16
                marker.scale.z = 0.1

                marker.pose.position.z = 0.05

                marker.color.r = 0.8
                marker.color.g = 0.8
                marker.color.b = 0.8
                marker.color.a = 1.0

            elif obj_type == 'S':

                marker.type = Marker.SPHERE
                marker.ns = "S"

                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.15

                marker.color.b = 1.0
                marker.color.a = 1.0

            ma.markers.append(marker)

        self.marker_pub.publish(ma)

    # ======================================================
    # Publish objects
    # ======================================================

    def publish_objects(self):

        static_transforms = []

        msg = ObjectList()

        for i, (ox, oy, angle) in enumerate(self.object_coords):

            if self.object_types[i] in ['O', 'B']:

                obj = Object()

                obj.id = str(i)

                obj.type = self.object_types[i]

                obj.pose.position.x = ox
                obj.pose.position.y = oy

                t = TransformStamped()

                t.header.stamp = self.get_clock().now().to_msg()

                t.header.frame_id = 'map'

                if self.object_types[i] == 'B':
                    t.child_frame_id = f"Box_{i}"

                else:
                    t.child_frame_id = f"Cube_{i}"

                t.transform.translation.x = float(ox)
                t.transform.translation.y = float(oy)

                q = quaternion_from_euler(
                    0.0,
                    0.0,
                    angle
                )

                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                static_transforms.append(t)

                msg.objects.append(obj)

        self.tf_static_broadcaster.sendTransform(
            static_transforms
        )

        self.object_pub.publish(msg)

    # ======================================================
    # Workspace
    # ======================================================

    def generate_workspace(self):

        self.workspace_poly = Polygon(self.workspace)

        grid = np.full(
            (self.height, self.width),
            100,
            dtype=np.int8
        )

        for r in range(self.height):

            for c in range(self.width):

                x = (
                    c * self.resolution +
                    self.origin_x
                )

                y = (
                    r * self.resolution +
                    self.origin_y
                )

                if self.workspace_poly.contains(
                    ShapePoint(x, y)
                ):

                    grid[r, c] = -1

        return grid

    # ======================================================
    # Save CSV
    # ======================================================

    def sync_full_map_to_csv(self):

        try:

            new_path = "/home/snowwhite/dd2419_ws/src/mapping/map/test.csv"

            with open(new_path, 'w') as f:

                f.write("type,x,y,angle\n")

                for i in range(len(self.object_types)):

                    ox, oy, ang = self.object_coords[i]

                    f.write(
                        f"{self.object_types[i]},"
                        f"{ox*100:.2f},"
                        f"{oy*100:.2f},"
                        f"{ang:.2f}\n"
                    )

            self.get_logger().info(
                "Map sync successful."
            )

        except Exception as e:

            self.get_logger().error(
                f"Error syncing map: {e}"
            )


def main():

    rclpy.init()

    time.sleep(1.5)

    node = GridPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
