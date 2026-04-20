from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import Point
from manus_system.msg import ManusGloveRaw, ManusRawNode
import numpy as np
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .manus_mapping import HandMetadata, ManusNodeMetadata


MEDIAPIPE_CONNECTIONS: tuple[tuple[int, int], ...] = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
)

LANDMARK_COLORS: tuple[ColorRGBA, ...] = (
    ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
    ColorRGBA(r=1.0, g=0.4, b=0.0, a=1.0),
    ColorRGBA(r=1.0, g=0.4, b=0.0, a=1.0),
    ColorRGBA(r=1.0, g=0.4, b=0.0, a=1.0),
    ColorRGBA(r=1.0, g=0.4, b=0.0, a=1.0),
    ColorRGBA(r=0.1, g=0.9, b=0.1, a=1.0),
    ColorRGBA(r=0.1, g=0.9, b=0.1, a=1.0),
    ColorRGBA(r=0.1, g=0.9, b=0.1, a=1.0),
    ColorRGBA(r=0.1, g=0.9, b=0.1, a=1.0),
    ColorRGBA(r=0.2, g=0.6, b=1.0, a=1.0),
    ColorRGBA(r=0.2, g=0.6, b=1.0, a=1.0),
    ColorRGBA(r=0.2, g=0.6, b=1.0, a=1.0),
    ColorRGBA(r=0.2, g=0.6, b=1.0, a=1.0),
    ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0),
    ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0),
    ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0),
    ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0),
    ColorRGBA(r=1.0, g=0.1, b=0.8, a=1.0),
    ColorRGBA(r=1.0, g=0.1, b=0.8, a=1.0),
    ColorRGBA(r=1.0, g=0.1, b=0.8, a=1.0),
    ColorRGBA(r=1.0, g=0.1, b=0.8, a=1.0),
)

RAW_NODE_COLORS = {
    "hand": ColorRGBA(r=0.9, g=0.9, b=0.9, a=1.0),
    "thumb": ColorRGBA(r=1.0, g=0.4, b=0.0, a=1.0),
    "index": ColorRGBA(r=0.1, g=0.9, b=0.1, a=1.0),
    "middle": ColorRGBA(r=0.2, g=0.6, b=1.0, a=1.0),
    "ring": ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0),
    "pinky": ColorRGBA(r=1.0, g=0.1, b=0.8, a=1.0),
}
RAW_NODE_DEFAULT_COLOR = ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0)

MIN_LABEL_HEIGHT_OFFSET = 0.015
MIN_LABEL_SCALE = 0.02
MIN_RAW_POINT_SCALE = 0.012
MIN_RAW_LINE_WIDTH = 0.003
MIN_RAW_LABEL_SCALE = 0.018


@dataclass(slots=True)
class KeypointMarkerConfig:
    left_topic: str
    right_topic: str
    left_frame_id: str
    right_frame_id: str
    align_wrist_to_origin: bool
    point_scale: float
    line_width: float
    lifetime_sec: float


def _duration_components(lifetime_sec: float) -> tuple[int, int]:
    return int(lifetime_sec), int((lifetime_sec % 1.0) * 1e9)


def _set_lifetime(marker: Marker, lifetime_sec: float) -> None:
    marker.lifetime.sec, marker.lifetime.nanosec = _duration_components(lifetime_sec)


def _point(point: np.ndarray, z_offset: float = 0.0) -> Point:
    return Point(x=float(point[0]), y=float(point[1]), z=float(point[2] + z_offset))


def _connection_color(start: int, end: int) -> ColorRGBA:
    if start == 0:
        return LANDMARK_COLORS[end]
    return LANDMARK_COLORS[start]


def _format_label(prefix: str, index: int, metadata: Optional[ManusNodeMetadata]) -> str:
    if metadata is None:
        return f"{prefix} mp={index} | missing"
    return (
        f"{prefix} mp={index} | id={metadata.node_id} | parent={metadata.parent_node_id} | "
        f"chain={metadata.chain_type} | joint={metadata.joint_type}"
    )


def _raw_color(chain_type: str) -> ColorRGBA:
    return RAW_NODE_COLORS.get(str(chain_type).strip().lower(), RAW_NODE_DEFAULT_COLOR)


def _raw_label(prefix: str, node: ManusRawNode) -> str:
    return (
        f"{prefix} raw id={int(node.node_id)} | parent={int(node.parent_node_id)} | "
        f"chain={node.chain_type} | joint={node.joint_type}"
    )


def _raw_position(node: ManusRawNode) -> np.ndarray:
    return np.array(
        [
            float(node.pose.position.x),
            float(node.pose.position.y),
            float(node.pose.position.z),
        ],
        dtype=np.float32,
    )


def _raw_node_map(glove: ManusGloveRaw) -> dict[int, ManusRawNode]:
    return {int(node.node_id): node for node in glove.raw_nodes}


def _find_glove(gloves: list[ManusGloveRaw] | tuple[ManusGloveRaw, ...], side: str) -> ManusGloveRaw | None:
    for glove in gloves:
        if str(glove.side).strip().lower() == side:
            return glove
    return None


class KeypointMarkerPublisher:
    def __init__(self, node, config: KeypointMarkerConfig) -> None:
        self._node = node
        self._config = config
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._left_pub = node.create_publisher(MarkerArray, config.left_topic, marker_qos)
        self._right_pub = node.create_publisher(MarkerArray, config.right_topic, marker_qos)

    def publish(
        self,
        *,
        left_keypoints: np.ndarray | None,
        right_keypoints: np.ndarray | None,
        left_metadata: Optional[HandMetadata] = None,
        right_metadata: Optional[HandMetadata] = None,
    ) -> None:
        if left_keypoints is not None:
            self._left_pub.publish(
                self._build_marker_array(
                    keypoints=left_keypoints,
                    metadata=left_metadata,
                    frame_id=self._config.left_frame_id,
                    namespace="left_keypoints",
                    label_prefix="L",
                )
            )
        if right_keypoints is not None:
            self._right_pub.publish(
                self._build_marker_array(
                    keypoints=right_keypoints,
                    metadata=right_metadata,
                    frame_id=self._config.right_frame_id,
                    namespace="right_keypoints",
                    label_prefix="R",
                )
            )

    def _build_marker_array(
        self,
        *,
        keypoints: np.ndarray,
        metadata: Optional[HandMetadata],
        frame_id: str,
        namespace: str,
        label_prefix: str,
    ) -> MarkerArray:
        points = np.asarray(keypoints, dtype=np.float32).reshape(21, 3)
        if self._config.align_wrist_to_origin:
            points = points - points[0:1, :]

        now = self._node.get_clock().now().to_msg()
        label_height_offset = max(self._config.point_scale * 0.75, MIN_LABEL_HEIGHT_OFFSET)
        label_scale = max(self._config.point_scale, MIN_LABEL_SCALE)

        point_marker = Marker()
        point_marker.header.frame_id = frame_id
        point_marker.header.stamp = now
        point_marker.ns = f"{namespace}_points"
        point_marker.id = 0
        point_marker.type = Marker.SPHERE_LIST
        point_marker.action = Marker.ADD
        point_marker.pose.orientation.w = 1.0
        point_marker.scale.x = self._config.point_scale
        point_marker.scale.y = self._config.point_scale
        point_marker.scale.z = self._config.point_scale
        point_marker.color.a = 1.0
        _set_lifetime(point_marker, self._config.lifetime_sec)
        point_marker.points = [_point(point) for point in points]
        point_marker.colors = list(LANDMARK_COLORS)

        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.header.stamp = now
        line_marker.ns = f"{namespace}_lines"
        line_marker.id = 1
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = self._config.line_width
        _set_lifetime(line_marker, self._config.lifetime_sec)
        line_marker.points = []
        line_marker.colors = []
        for start, end in MEDIAPIPE_CONNECTIONS:
            color = _connection_color(start, end)
            line_marker.points.append(_point(points[start]))
            line_marker.points.append(_point(points[end]))
            line_marker.colors.append(color)
            line_marker.colors.append(color)

        text_markers: list[Marker] = []
        marker_metadata = metadata if metadata is not None else tuple([None] * len(points))
        for index, point in enumerate(points):
            text_marker = Marker()
            text_marker.header.frame_id = frame_id
            text_marker.header.stamp = now
            text_marker.ns = f"{namespace}_labels"
            text_marker.id = 100 + index
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = _point(point, label_height_offset)
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = label_scale
            text_marker.color = LANDMARK_COLORS[index]
            text_marker.text = _format_label(label_prefix, index, marker_metadata[index])
            _set_lifetime(text_marker, self._config.lifetime_sec)
            text_markers.append(text_marker)

        return MarkerArray(markers=[point_marker, line_marker, *text_markers])


class RawManusMarkerPublisher:
    def __init__(self, node, config: KeypointMarkerConfig) -> None:
        self._node = node
        self._config = config
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._left_pub = node.create_publisher(MarkerArray, f"{config.left_topic}_raw", marker_qos)
        self._right_pub = node.create_publisher(MarkerArray, f"{config.right_topic}_raw", marker_qos)

    def publish(self, gloves: list[ManusGloveRaw] | tuple[ManusGloveRaw, ...]) -> None:
        now = self._node.get_clock().now().to_msg()
        left_glove = _find_glove(gloves, "left")
        right_glove = _find_glove(gloves, "right")
        if left_glove is not None:
            self._left_pub.publish(
                self._build_marker_array(left_glove, self._config.left_frame_id, now, "L")
            )
        if right_glove is not None:
            self._right_pub.publish(
                self._build_marker_array(right_glove, self._config.right_frame_id, now, "R")
            )

    def _build_marker_array(
        self,
        glove: ManusGloveRaw,
        frame_id: str,
        stamp,
        label_prefix: str,
    ) -> MarkerArray:
        node_map = _raw_node_map(glove)
        point_scale = max(self._config.point_scale * 0.7, MIN_RAW_POINT_SCALE)
        line_width = max(self._config.line_width * 0.5, MIN_RAW_LINE_WIDTH)
        label_scale = max(self._config.point_scale * 0.9, MIN_RAW_LABEL_SCALE)

        point_marker = Marker()
        point_marker.header.frame_id = frame_id
        point_marker.header.stamp = stamp
        point_marker.ns = f"{label_prefix}_raw_points"
        point_marker.id = 0
        point_marker.type = Marker.SPHERE_LIST
        point_marker.action = Marker.ADD
        point_marker.pose.orientation.w = 1.0
        point_marker.scale.x = point_scale
        point_marker.scale.y = point_scale
        point_marker.scale.z = point_scale
        point_marker.color.a = 1.0
        _set_lifetime(point_marker, self._config.lifetime_sec)
        point_marker.points = []
        point_marker.colors = []

        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.header.stamp = stamp
        line_marker.ns = f"{label_prefix}_raw_lines"
        line_marker.id = 1
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = line_width
        _set_lifetime(line_marker, self._config.lifetime_sec)
        line_marker.points = []
        line_marker.colors = []

        label_markers: list[Marker] = []
        for node in sorted(glove.raw_nodes, key=lambda item: int(item.node_id)):
            color = _raw_color(node.chain_type)
            position = _raw_position(node)
            point_marker.points.append(_point(position))
            point_marker.colors.append(color)

            parent_id = int(node.parent_node_id)
            if parent_id in node_map:
                parent_position = _raw_position(node_map[parent_id])
                line_marker.points.append(_point(parent_position))
                line_marker.points.append(_point(position))
                line_marker.colors.append(color)
                line_marker.colors.append(color)

            label_marker = Marker()
            label_marker.header.frame_id = frame_id
            label_marker.header.stamp = stamp
            label_marker.ns = f"{label_prefix}_raw_labels"
            label_marker.id = 1000 + int(node.node_id)
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.pose.position = _point(position, MIN_LABEL_HEIGHT_OFFSET)
            label_marker.pose.orientation.w = 1.0
            label_marker.scale.z = label_scale
            label_marker.color = color
            label_marker.text = _raw_label(label_prefix, node)
            _set_lifetime(label_marker, self._config.lifetime_sec)
            label_markers.append(label_marker)

        return MarkerArray(markers=[point_marker, line_marker, *label_markers])

    @property
    def left_topic(self) -> str:
        return f"{self._config.left_topic}_raw"

    @property
    def right_topic(self) -> str:
        return f"{self._config.right_topic}_raw"
