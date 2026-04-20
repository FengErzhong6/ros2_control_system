"""MediaPipe format conversion utilities for hand pose data."""

import numpy as np


_FRAME_EPS = 1e-8


class MediaPipeSmoother:
    """Maintains independent MediaPipe format data smoothing buffer for each hand."""
    def __init__(self, buffer_size=5):
        self.buffer_size = buffer_size
        self.mediapipe_pose_buffer = None
    
    def smooth(self, mediapipe_pose):
        """Apply multi-frame smoothing to MediaPipe format pose data."""
        if self.mediapipe_pose_buffer is None:
            self.mediapipe_pose_buffer = [mediapipe_pose.copy() for _ in range(self.buffer_size)]
            return mediapipe_pose

        self.mediapipe_pose_buffer.append(mediapipe_pose.copy())

        if len(self.mediapipe_pose_buffer) > self.buffer_size:
            self.mediapipe_pose_buffer.pop(0)

        n = len(self.mediapipe_pose_buffer)
        if n == 1:
            weights = np.array([1.0])
        else:
            weights = np.exp(np.linspace(-2.0, 0.0, n))
            weights = weights / np.sum(weights)

        smoothed_pose = np.zeros_like(mediapipe_pose)
        for frame, weight in zip(self.mediapipe_pose_buffer, weights):
            smoothed_pose += weight * frame

        return smoothed_pose


# Coordinate transformation matrices for MANO hand model
OPERATOR2MANO_RIGHT = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
])

OPERATOR2MANO_LEFT = np.array([
    [0, 0, -1],
    [1, 0, 0],
    [0, -1, 0],
])


def estimate_frame_from_hand_points(keypoint_3d_array: np.ndarray) -> np.ndarray:
    """
    Compute the 3D coordinate frame (orientation only) from detected 3d key points.
    
    Args:
        keypoint_3d_array: keypoint3 detected from hand detector. Shape: (21, 3)
        
    Returns:
        frame: the coordinate frame of wrist in MANO convention
    """
    keypoint_3d_array = np.asarray(keypoint_3d_array, dtype=np.float64)
    if keypoint_3d_array.shape != (21, 3):
        raise ValueError(f"Expected keypoints with shape (21, 3), got {keypoint_3d_array.shape}.")
    if not np.all(np.isfinite(keypoint_3d_array)):
        raise ValueError("Keypoints contain non-finite values.")

    points = keypoint_3d_array[[0, 5, 9], :]

    x_vector = points[0] - points[2]
    if np.linalg.norm(x_vector) <= _FRAME_EPS:
        raise ValueError("Degenerate hand frame: wrist and middle MCP are too close.")

    points = points - np.mean(points, axis=0, keepdims=True)
    if np.max(np.linalg.norm(points, axis=1)) <= _FRAME_EPS:
        raise ValueError("Degenerate hand frame: reference points collapse to a single point.")
    try:
        _, _, v = np.linalg.svd(points)
    except np.linalg.LinAlgError as exc:
        raise ValueError("Failed to estimate hand frame from keypoints.") from exc

    normal = v[2, :]
    if not np.all(np.isfinite(normal)) or np.linalg.norm(normal) <= _FRAME_EPS:
        raise ValueError("Degenerate hand frame: invalid plane normal.")

    x = x_vector - np.sum(x_vector * normal) * normal
    x_norm = np.linalg.norm(x)
    if x_norm <= _FRAME_EPS:
        raise ValueError("Degenerate hand frame: x-axis projection is too small.")
    x = x / x_norm
    z = np.cross(x, normal)
    z_norm = np.linalg.norm(z)
    if z_norm <= _FRAME_EPS:
        raise ValueError("Degenerate hand frame: z-axis projection is too small.")
    z = z / z_norm

    if np.sum(z * (points[1] - points[2])) < 0:
        normal *= -1
        z *= -1
    frame = np.stack([x, normal, z], axis=1)
    return frame


def apply_mediapipe_transformations(keypoint_3d_array: np.ndarray, hand_type: str = "right") -> np.ndarray:
    """
    Apply the same coordinate transformations as MediaPipe data processing.
    
    Args:
        keypoint_3d_array: numpy array of shape (21, 3) - hand landmarks
        hand_type: "right" or "left" - determines coordinate system
        
    Returns:
        transformed_joint_pos: numpy array of shape (21, 3) - transformed landmarks
    """
    hand_type = hand_type.lower()
    
    keypoint_3d_array = keypoint_3d_array - keypoint_3d_array[0:1, :]
    
    mediapipe_wrist_rot = estimate_frame_from_hand_points(keypoint_3d_array)
    
    operator2mano = OPERATOR2MANO_RIGHT if hand_type == "right" else OPERATOR2MANO_LEFT
    joint_pos = keypoint_3d_array @ mediapipe_wrist_rot @ operator2mano
    
    return joint_pos


__all__ = [
    'MediaPipeSmoother',
    'OPERATOR2MANO_RIGHT',
    'OPERATOR2MANO_LEFT',
    'estimate_frame_from_hand_points',
    'apply_mediapipe_transformations',
]
