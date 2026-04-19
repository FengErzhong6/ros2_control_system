from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np

# MediaPipe order: wrist, thumb(4), index(4), middle(4), ring(4), pinky(4)
MEDIAPIPE_TO_MANUS: Tuple[int, ...] = (
    1, 22, 23, 24, 25,
    3, 4, 5, 6,
    8, 9, 10, 11,
    13, 14, 15, 16,
    18, 19, 20, 21,
)

SINGLE_HAND_VALUES = 21 * 3
DUAL_HAND_VALUES = SINGLE_HAND_VALUES * 2
_MIN_REQUIRED_MP_INDICES = (0, 5, 9)
_MIN_NORM = 1e-6


@dataclass(frozen=True, slots=True)
class MappingOptions:
    flip_x: bool = False
    flip_y: bool = True
    flip_z: bool = False
    expected_left_side_name: str = "left"
    expected_right_side_name: str = "right"


def normalize_side_name(value: str) -> str:
    return str(value).strip().lower()


def _apply_axis_flips(position: np.ndarray, options: MappingOptions) -> np.ndarray:
    if options.flip_x:
        position[0] = -position[0]
    if options.flip_y:
        position[1] = -position[1]
    if options.flip_z:
        position[2] = -position[2]
    return position


def glove_to_mediapipe(glove: object, options: MappingOptions) -> np.ndarray:
    manus_positions: dict[int, np.ndarray] = {}
    for raw_node in getattr(glove, "raw_nodes", []):
        position = np.array(
            [
                raw_node.pose.position.x,
                raw_node.pose.position.y,
                raw_node.pose.position.z,
            ],
            dtype=np.float32,
        )
        manus_positions[int(raw_node.node_id)] = _apply_axis_flips(position, options)

    mediapipe_pose = np.zeros((21, 3), dtype=np.float32)
    for mp_idx, manus_node_id in enumerate(MEDIAPIPE_TO_MANUS):
        if manus_node_id in manus_positions:
            mediapipe_pose[mp_idx] = manus_positions[manus_node_id]
    return mediapipe_pose


def extract_hands_from_gloves(
    gloves: Iterable[object],
    options: MappingOptions,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    left_keypoints: Optional[np.ndarray] = None
    right_keypoints: Optional[np.ndarray] = None

    expected_left = normalize_side_name(options.expected_left_side_name)
    expected_right = normalize_side_name(options.expected_right_side_name)

    for glove in gloves:
        side = normalize_side_name(getattr(glove, "side", ""))
        if side == expected_left:
            left_keypoints = glove_to_mediapipe(glove, options)
        elif side == expected_right:
            right_keypoints = glove_to_mediapipe(glove, options)

    return left_keypoints, right_keypoints


def flatten_hand_input(
    left_keypoints: Optional[np.ndarray],
    right_keypoints: Optional[np.ndarray],
    *,
    include_left_hand: bool,
    include_right_hand: bool,
    drop_if_missing_hand: bool,
    right_first: bool,
) -> Optional[np.ndarray]:
    payloads: list[np.ndarray] = []
    order = [
        ("right", right_keypoints, include_right_hand),
        ("left", left_keypoints, include_left_hand),
    ]
    if not right_first:
        order.reverse()

    for _, keypoints, enabled in order:
        if not enabled:
            continue
        if keypoints is None:
            if drop_if_missing_hand:
                return None
            continue
        payloads.append(np.asarray(keypoints, dtype=np.float32).reshape(-1))

    if not payloads:
        return None
    return np.concatenate(payloads).astype(np.float32, copy=False)


def split_hand_input(
    payload: Sequence[float] | np.ndarray,
    *,
    enable_left_hand: bool,
    enable_right_hand: bool,
    single_hand_fallback_side: str,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    flat = np.asarray(payload, dtype=np.float32).reshape(-1)
    fallback = normalize_side_name(single_hand_fallback_side)

    if enable_left_hand and enable_right_hand:
        if flat.size == DUAL_HAND_VALUES:
            right = flat[:SINGLE_HAND_VALUES].reshape(21, 3)
            left = flat[SINGLE_HAND_VALUES:].reshape(21, 3)
            return left, right
        if flat.size == SINGLE_HAND_VALUES:
            single = flat.reshape(21, 3)
            if fallback == "left":
                return single, None
            return None, single
        raise ValueError(
            f"Expected {SINGLE_HAND_VALUES} or {DUAL_HAND_VALUES} values, got {flat.size}."
        )

    if enable_left_hand and not enable_right_hand:
        if flat.size != SINGLE_HAND_VALUES:
            raise ValueError(f"Expected {SINGLE_HAND_VALUES} values for left hand, got {flat.size}.")
        return flat.reshape(21, 3), None

    if enable_right_hand and not enable_left_hand:
        if flat.size != SINGLE_HAND_VALUES:
            raise ValueError(f"Expected {SINGLE_HAND_VALUES} values for right hand, got {flat.size}.")
        return None, flat.reshape(21, 3)

    raise ValueError("Both left and right hand outputs are disabled.")


def is_valid_hand_keypoints(keypoints: Optional[np.ndarray]) -> bool:
    if keypoints is None:
        return False
    array = np.asarray(keypoints, dtype=np.float32)
    if array.shape != (21, 3):
        return False
    if not np.all(np.isfinite(array)):
        return False

    required = array[list(_MIN_REQUIRED_MP_INDICES)]
    if np.linalg.norm(required[1] - required[0]) <= _MIN_NORM:
        return False
    if np.linalg.norm(required[2] - required[0]) <= _MIN_NORM:
        return False

    nonzero_points = np.sum(np.linalg.norm(array, axis=1) > _MIN_NORM)
    return int(nonzero_points) >= 5
