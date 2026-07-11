#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster


class TagLocalizationNode(Node):
    def __init__(self):
        super().__init__('tag_localization_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('marker_size', 0.007)
        self.declare_parameter('marker_gap', 0.003)
        self.declare_parameter('marker_pitch', 0.010)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('horizontal_fov', 1.047)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('marker_frame', 'marker_0')
        self.declare_parameter('estimated_camera_frame', 'estimated_camera_link')
        self.declare_parameter('estimated_base_frame', 'estimated_base_link')
        self.declare_parameter('use_grid_layout', True)
        self.declare_parameter('grid_origin_x', -0.510715)
        self.declare_parameter('grid_origin_y', 0.350583)
        self.declare_parameter('grid_origin_z', 1.99)
        self.declare_parameter('grid_cols', 100)
        self.declare_parameter('marker_map_x', -0.510715)
        self.declare_parameter('marker_map_y', 0.350583)
        self.declare_parameter('marker_map_z', 1.99)
        self.declare_parameter('marker_map_roll', 3.14159)
        self.declare_parameter('marker_map_pitch', 0.0)
        self.declare_parameter('marker_map_yaw', 0.0)
        self.declare_parameter('base_to_camera_x', 0.0)
        self.declare_parameter('base_to_camera_y', 0.0)
        self.declare_parameter('base_to_camera_z', 0.020)
        self.declare_parameter('base_to_camera_qx', 0.0)
        self.declare_parameter('base_to_camera_qy', -0.70710678)
        self.declare_parameter('base_to_camera_qz', 0.0)
        self.declare_parameter('base_to_camera_qw', 0.70710678)

        self.image_topic = self.get_parameter('image_topic').value
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.marker_gap = float(self.get_parameter('marker_gap').value)
        self.marker_pitch = float(self.get_parameter('marker_pitch').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.horizontal_fov = float(self.get_parameter('horizontal_fov').value)
        self.camera_frame = self.get_parameter('camera_frame').value
        self.publish_tf_enabled = bool(self.get_parameter('publish_tf').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value
        self.estimated_camera_frame = self.get_parameter('estimated_camera_frame').value
        self.estimated_base_frame = self.get_parameter('estimated_base_frame').value
        self.use_grid_layout = bool(self.get_parameter('use_grid_layout').value)
        self.grid_origin_x = float(self.get_parameter('grid_origin_x').value)
        self.grid_origin_y = float(self.get_parameter('grid_origin_y').value)
        self.grid_origin_z = float(self.get_parameter('grid_origin_z').value)
        self.grid_cols = int(self.get_parameter('grid_cols').value)

        self.marker_map_x = float(self.get_parameter('marker_map_x').value)
        self.marker_map_y = float(self.get_parameter('marker_map_y').value)
        self.marker_map_z = float(self.get_parameter('marker_map_z').value)
        self.marker_map_roll = float(self.get_parameter('marker_map_roll').value)
        self.marker_map_pitch = float(self.get_parameter('marker_map_pitch').value)
        self.marker_map_yaw = float(self.get_parameter('marker_map_yaw').value)
        self.base_to_camera_x = float(self.get_parameter('base_to_camera_x').value)
        self.base_to_camera_y = float(self.get_parameter('base_to_camera_y').value)
        self.base_to_camera_z = float(self.get_parameter('base_to_camera_z').value)
        self.base_to_camera_qx = float(self.get_parameter('base_to_camera_qx').value)
        self.base_to_camera_qy = float(self.get_parameter('base_to_camera_qy').value)
        self.base_to_camera_qz = float(self.get_parameter('base_to_camera_qz').value)
        self.base_to_camera_qw = float(self.get_parameter('base_to_camera_qw').value)

        self.bridge = CvBridge()
        self.camera_matrix = self.compute_camera_matrix(
            self.image_width,
            self.image_height,
            self.horizontal_fov,
        )
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        self.pose_pub = self.create_publisher(PoseStamped, '/detected_marker_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf_enabled else None

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h10)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h10)

        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.use_new_api = hasattr(cv2.aruco, 'ArucoDetector')
        if self.use_new_api:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.get_logger().info('Using new OpenCV ArucoDetector API.')
        else:
            self.detector = None
            self.get_logger().info('Using old OpenCV detectMarkers API.')

        self.frame_count = 0
        self.base_to_camera_transform = self.make_transform_matrix(
            self.quaternion_to_rotation_matrix(
                self.base_to_camera_qx,
                self.base_to_camera_qy,
                self.base_to_camera_qz,
                self.base_to_camera_qw,
            ),
            np.array(
                [
                    self.base_to_camera_x,
                    self.base_to_camera_y,
                    self.base_to_camera_z,
                ],
                dtype=np.float64,
            ),
        )

        self.get_logger().info('Marker localization node started.')
        self.get_logger().info(f'Subscribed image topic: {self.image_topic}')
        self.get_logger().info('Using dictionary: DICT_APRILTAG_36h10')
        self.get_logger().info(f'Marker size: {self.marker_size} m')
        self.get_logger().info(f'Marker gap: {self.marker_gap} m')
        self.get_logger().info(f'Marker pitch: {self.marker_pitch} m')
        self.get_logger().info(f'Image size: {self.image_width} x {self.image_height}')
        self.get_logger().info(f'Horizontal FOV: {self.horizontal_fov} rad')
        self.get_logger().info(f'Camera frame: {self.camera_frame}')
        self.get_logger().info(f'Camera matrix:\n{self.camera_matrix}')

    def compute_camera_matrix(self, width, height, horizontal_fov):
        fx = width / (2.0 * math.tan(horizontal_fov / 2.0))
        fy = fx
        cx = width / 2.0
        cy = height / 2.0
        return np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def rotation_matrix_to_quaternion(self, rotation):
        trace = np.trace(rotation)

        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (rotation[2, 1] - rotation[1, 2]) / s
            qy = (rotation[0, 2] - rotation[2, 0]) / s
            qz = (rotation[1, 0] - rotation[0, 1]) / s
        elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / s
            qx = 0.25 * s
            qy = (rotation[0, 1] + rotation[1, 0]) / s
            qz = (rotation[0, 2] + rotation[2, 0]) / s
        elif rotation[1, 1] > rotation[2, 2]:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / s
            qx = (rotation[0, 1] + rotation[1, 0]) / s
            qy = 0.25 * s
            qz = (rotation[1, 2] + rotation[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / s
            qx = (rotation[0, 2] + rotation[2, 0]) / s
            qy = (rotation[1, 2] + rotation[2, 1]) / s
            qz = 0.25 * s

        return float(qx), float(qy), float(qz), float(qw)

    def rpy_to_rotation_matrix(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
        rot_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
        rot_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        return rot_z @ rot_y @ rot_x

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0:
            self.get_logger().warn('Invalid base_to_camera quaternion norm. Falling back to identity rotation.')
            return np.eye(3, dtype=np.float64)

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        return np.array(
            [
                [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
                [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
                [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
            ],
            dtype=np.float64,
        )

    def make_transform_matrix(self, rotation, translation):
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation
        return transform

    def invert_transform(self, transform):
        rotation = transform[:3, :3]
        translation = transform[:3, 3]
        inverse = np.eye(4, dtype=np.float64)
        inverse[:3, :3] = rotation.T
        inverse[:3, 3] = -rotation.T @ translation
        return inverse

    def publish_transform(self, transform, parent_frame, child_frame, stamp):
        if self.tf_broadcaster is None:
            return

        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = float(transform[0, 3])
        msg.transform.translation.y = float(transform[1, 3])
        msg.transform.translation.z = float(transform[2, 3])

        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(transform[:3, :3])
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(msg)

    def get_marker_map_pose(self, marker_id):
        if self.use_grid_layout:
            safe_grid_cols = max(1, self.grid_cols)
            row = marker_id // safe_grid_cols
            col = marker_id % safe_grid_cols
            x = self.grid_origin_x + col * self.marker_pitch
            y = self.grid_origin_y + row * self.marker_pitch
            z = self.grid_origin_z
        else:
            row = 0
            col = 0
            x = self.marker_map_x
            y = self.marker_map_y
            z = self.marker_map_z

        return {
            'row': row,
            'col': col,
            'x': x,
            'y': y,
            'z': z,
            'roll': self.marker_map_roll,
            'pitch': self.marker_map_pitch,
            'yaw': self.marker_map_yaw,
        }

    def publish_map_to_camera_tf(self, marker_id, rvec, tvec, stamp):
        if not self.publish_tf_enabled:
            return

        marker_pose = self.get_marker_map_pose(marker_id)
        marker_map_transform = self.make_transform_matrix(
            self.rpy_to_rotation_matrix(
                marker_pose['roll'],
                marker_pose['pitch'],
                marker_pose['yaw'],
            ),
            np.array([marker_pose['x'], marker_pose['y'], marker_pose['z']], dtype=np.float64),
        )

        rotation_camera_marker, _ = cv2.Rodrigues(rvec.reshape(3, 1))
        transform_camera_marker = self.make_transform_matrix(rotation_camera_marker, tvec)
        transform_map_camera = marker_map_transform @ self.invert_transform(transform_camera_marker)

        self.publish_transform(
            transform_map_camera,
            self.map_frame,
            self.estimated_camera_frame,
            stamp,
        )
        transform_map_base = transform_map_camera @ self.invert_transform(self.base_to_camera_transform)
        self.publish_transform(
            transform_map_base,
            self.map_frame,
            self.estimated_base_frame,
            stamp,
        )

        self.get_logger().info(
            f'marker_id={marker_id} row={marker_pose["row"]} col={marker_pose["col"]} '
            f'marker_map_pose=[{marker_pose["x"]:.4f}, {marker_pose["y"]:.4f}, {marker_pose["z"]:.4f}]'
        )
        self.get_logger().info(
            'map -> camera: '
            f'x={transform_map_camera[0, 3]:.4f}, '
            f'y={transform_map_camera[1, 3]:.4f}, '
            f'z={transform_map_camera[2, 3]:.4f}'
        )
        self.get_logger().info(
            'map -> base: '
            f'x={transform_map_base[0, 3]:.4f}, '
            f'y={transform_map_base[1, 3]:.4f}, '
            f'z={transform_map_base[2, 3]:.4f}'
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        self.frame_count += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        try:
            if self.use_new_api:
                corners, ids, _rejected = self.detector.detectMarkers(gray)
            else:
                corners, ids, _rejected = cv2.aruco.detectMarkers(
                    gray,
                    self.aruco_dict,
                    parameters=self.aruco_params,
                )
        except Exception as exc:
            self.get_logger().error(f'Marker detection error: {exc}')
            return

        if ids is None or len(corners) == 0:
            if self.frame_count % 30 == 0:
                height, width = frame.shape[:2]
                self.get_logger().info(
                    f'Image received but no marker detected. width={width}, height={height}'
                )
            return

        detected_ids = ids.flatten().tolist()
        marker_id = int(detected_ids[0])
        marker_corners = corners[0]

        rvec, tvec = self.estimate_single_marker_pose(marker_corners)
        if rvec is None or tvec is None:
            return

        pose_msg = self.make_pose_msg(rvec, tvec, msg.header.stamp)
        self.pose_pub.publish(pose_msg)

        self.get_logger().info(
            f'ID={marker_id} '
            f'tvec[m]=[{tvec[0]:.4f}, {tvec[1]:.4f}, {tvec[2]:.4f}] '
            f'rvec=[{rvec[0]:.4f}, {rvec[1]:.4f}, {rvec[2]:.4f}]'
        )

        self.publish_map_to_camera_tf(marker_id, rvec, tvec, msg.header.stamp)

    def estimate_single_marker_pose(self, marker_corners):
        half_size = self.marker_size / 2.0
        object_points = np.array(
            [
                [-half_size, half_size, 0.0],
                [half_size, half_size, 0.0],
                [half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0],
            ],
            dtype=np.float64,
        )
        image_points = marker_corners.reshape((4, 2)).astype(np.float64)

        pnp_flag = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)
        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=pnp_flag,
            )
            if not success and pnp_flag != cv2.SOLVEPNP_ITERATIVE:
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
        except Exception as exc:
            self.get_logger().error(f'solvePnP error: {exc}')
            return None, None

        if not success:
            self.get_logger().warn('solvePnP failed.')
            return None, None

        return rvec.flatten(), tvec.flatten()

    def make_pose_msg(self, rvec, tvec, stamp):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.camera_frame
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])

        rotation_matrix, _ = cv2.Rodrigues(rvec.reshape(3, 1))
        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(rotation_matrix)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        return pose_msg


def main(args=None):
    rclpy.init(args=args)
    node = TagLocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
