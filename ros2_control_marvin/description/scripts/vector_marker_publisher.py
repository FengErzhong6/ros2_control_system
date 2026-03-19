#!/usr/bin/env python3
"""
Publish visualization markers for vectors and planes.

Each arm (L / R) can have vectors and/or planes, all expressed in the
corresponding arm base frame (Base_L / Base_R).

Parameters (set via launch file or command line):
  vectors_L / vectors_R : str
      Semicolon-separated vectors.
      Per-vector format:  "vx vy vz"  or  "ox oy oz vx vy vz"

  planes_L / planes_R : str
      Semicolon-separated plane definitions (two non-collinear vectors).
      Per-plane format:  "v1x v1y v1z v2x v2y v2z"
                     or  "ox oy oz v1x v1y v1z v2x v2y v2z"
"""

import math
import sys

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA


_ARM_CONFIG = {
    "L": {"frame": "Base_L", "color": ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)},
    "R": {"frame": "Base_R", "color": ColorRGBA(r=0.2, g=0.5, b=1.0, a=1.0)},
}

_NORMAL_COLOR = ColorRGBA(r=0.2, g=0.9, b=0.2, a=1.0)


def _parse_vectors(raw: str):
    """Parse 'vx vy vz[;ox oy oz vx vy vz;...]' into list of (origin, direction)."""
    vectors = []
    for segment in raw.split(";"):
        parts = segment.split()
        if len(parts) == 3:
            origin = (0.0, 0.0, 0.0)
            direction = tuple(float(v) for v in parts)
        elif len(parts) == 6:
            origin = tuple(float(v) for v in parts[:3])
            direction = tuple(float(v) for v in parts[3:])
        else:
            continue
        vectors.append((origin, direction))
    return vectors


def _parse_planes(raw: str):
    """Parse plane definitions into list of (origin, v1, v2).

    6 values: v1x v1y v1z v2x v2y v2z  (origin = 0,0,0)
    9 values: ox oy oz v1x v1y v1z v2x v2y v2z
    """
    planes = []
    for segment in raw.split(";"):
        parts = segment.split()
        if len(parts) == 6:
            origin = (0.0, 0.0, 0.0)
            v1 = tuple(float(v) for v in parts[:3])
            v2 = tuple(float(v) for v in parts[3:6])
        elif len(parts) == 9:
            origin = tuple(float(v) for v in parts[:3])
            v1 = tuple(float(v) for v in parts[3:6])
            v2 = tuple(float(v) for v in parts[6:9])
        else:
            continue
        planes.append((origin, v1, v2))
    return planes


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def _scale(v, s):
    return (v[0] * s, v[1] * s, v[2] * s)


def _pt(x, y, z):
    return Point(x=float(x), y=float(y), z=float(z))


class VectorMarkerPublisher(Node):
    def __init__(self):
        super().__init__("vector_marker_publisher")

        self.declare_parameter("vectors_L", "")
        self.declare_parameter("vectors_R", "")
        self.declare_parameter("planes_L", "")
        self.declare_parameter("planes_R", "")
        self.declare_parameter("shaft_diameter", 0.006)
        self.declare_parameter("head_diameter", 0.012)
        self.declare_parameter("head_length", 0.015)

        self._pub = self.create_publisher(MarkerArray, "target_vectors", 10)
        self._markers = self._build_markers()

        if not self._markers.markers:
            self.get_logger().info("No vectors/planes provided – node will idle.")
            return

        self.get_logger().info(
            f"Publishing {len(self._markers.markers)} marker(s)."
        )
        self.create_timer(0.1, self._publish)

    def _make_arrow(self, frame_id, ns, mid, p0, p1, color):
        shaft_d = self.get_parameter("shaft_diameter").value
        head_d = self.get_parameter("head_diameter").value
        head_l = self.get_parameter("head_length").value
        m = Marker()
        m.header.frame_id = frame_id
        m.ns = ns
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.points.append(p0)
        m.points.append(p1)
        m.scale = Vector3(x=shaft_d, y=head_d, z=head_l)
        m.color = color
        return m

    def _build_markers(self) -> MarkerArray:
        ma = MarkerArray()
        mid = 0

        for suffix, cfg in _ARM_CONFIG.items():
            frame = cfg["frame"]
            color = cfg["color"]

            # --- vectors ---
            raw_vec = self.get_parameter(f"vectors_{suffix}").value
            if raw_vec:
                for origin, direction in _parse_vectors(raw_vec):
                    p0 = _pt(*origin)
                    p1 = _pt(origin[0] + direction[0],
                             origin[1] + direction[1],
                             origin[2] + direction[2])
                    ma.markers.append(
                        self._make_arrow(frame, f"vec_{suffix}", mid, p0, p1, color)
                    )
                    mid += 1

            # --- planes ---
            raw_plane = self.get_parameter(f"planes_{suffix}").value
            if not raw_plane:
                continue
            for origin, v1, v2 in _parse_planes(raw_plane):
                normal = _cross(v1, v2)
                n_len = _norm(normal)
                if n_len < 1e-9:
                    self.get_logger().warn("Skipping collinear plane vectors.")
                    continue

                o = origin
                a = (o[0] + v1[0], o[1] + v1[1], o[2] + v1[2])
                b = (o[0] + v1[0] + v2[0], o[1] + v1[1] + v2[1], o[2] + v1[2] + v2[2])
                c = (o[0] + v2[0], o[1] + v2[1], o[2] + v2[2])

                # parallelogram (two triangles, semi-transparent)
                tri = Marker()
                tri.header.frame_id = frame
                tri.ns = f"plane_{suffix}"
                tri.id = mid
                tri.type = Marker.TRIANGLE_LIST
                tri.action = Marker.ADD
                tri.scale = Vector3(x=1.0, y=1.0, z=1.0)
                plane_color = ColorRGBA(
                    r=color.r, g=color.g, b=color.b, a=0.25
                )
                for tri_pts in [(o, a, b), (o, b, c)]:
                    for p in tri_pts:
                        tri.points.append(_pt(*p))
                        tri.colors.append(plane_color)
                ma.markers.append(tri)
                mid += 1

                # v1 arrow
                ma.markers.append(self._make_arrow(
                    frame, f"plane_v1_{suffix}", mid,
                    _pt(*o), _pt(*a), color,
                ))
                mid += 1

                # v2 arrow
                ma.markers.append(self._make_arrow(
                    frame, f"plane_v2_{suffix}", mid,
                    _pt(*o), _pt(*c), color,
                ))
                mid += 1

                # normal arrow (scaled to half the average vector length)
                avg_len = (_norm(v1) + _norm(v2)) / 2.0
                n_unit = _scale(normal, 0.5 * avg_len / n_len)
                center = (
                    o[0] + (v1[0] + v2[0]) / 2.0,
                    o[1] + (v1[1] + v2[1]) / 2.0,
                    o[2] + (v1[2] + v2[2]) / 2.0,
                )
                ma.markers.append(self._make_arrow(
                    frame, f"plane_n_{suffix}", mid,
                    _pt(*center),
                    _pt(center[0] + n_unit[0],
                        center[1] + n_unit[1],
                        center[2] + n_unit[2]),
                    _NORMAL_COLOR,
                ))
                mid += 1

        return ma

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        for m in self._markers.markers:
            m.header.stamp = stamp
        self._pub.publish(self._markers)


def main():
    rclpy.init()
    node = VectorMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
