from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from wujihand_teleop.manus_mapping import (
    DUAL_HAND_VALUES,
    MappingOptions,
    SINGLE_HAND_VALUES,
    extract_hands_from_gloves,
    flatten_hand_input,
    glove_to_mediapipe,
    split_hand_input,
)


def _raw_node(node_id: int, x: float, y: float, z: float):
    return SimpleNamespace(
        node_id=node_id,
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z)),
    )


def _glove(side: str, nodes):
    return SimpleNamespace(side=side, raw_nodes=list(nodes))


def test_glove_to_mediapipe_maps_and_flips_y():
    glove = _glove(
        "left",
        [
            _raw_node(1, 1.0, 2.0, 3.0),
            _raw_node(22, 4.0, 5.0, 6.0),
            _raw_node(3, 7.0, 8.0, 9.0),
        ],
    )
    result = glove_to_mediapipe(glove, MappingOptions())
    assert result.shape == (21, 3)
    np.testing.assert_allclose(result[0], [1.0, -2.0, 3.0])
    np.testing.assert_allclose(result[1], [4.0, -5.0, 6.0])
    np.testing.assert_allclose(result[5], [7.0, -8.0, 9.0])


def test_extract_hands_and_flatten_right_first():
    left = _glove("left", [_raw_node(1, 1.0, 0.0, 0.0)])
    right = _glove("right", [_raw_node(1, 2.0, 0.0, 0.0)])
    left_keypoints, right_keypoints = extract_hands_from_gloves([left, right], MappingOptions())
    flat = flatten_hand_input(
        left_keypoints,
        right_keypoints,
        include_left_hand=True,
        include_right_hand=True,
        drop_if_missing_hand=True,
        right_first=True,
    )

    assert flat is not None
    assert flat.size == DUAL_HAND_VALUES
    np.testing.assert_allclose(flat[:3], [2.0, 0.0, 0.0])
    np.testing.assert_allclose(flat[SINGLE_HAND_VALUES:SINGLE_HAND_VALUES + 3], [1.0, 0.0, 0.0])


def test_split_hand_input_round_trip_dual():
    right = np.arange(SINGLE_HAND_VALUES, dtype=np.float32).reshape(21, 3)
    left = (1000.0 + np.arange(SINGLE_HAND_VALUES, dtype=np.float32)).reshape(21, 3)
    flat = np.concatenate([right.reshape(-1), left.reshape(-1)])
    left_out, right_out = split_hand_input(
        flat,
        enable_left_hand=True,
        enable_right_hand=True,
        single_hand_fallback_side="right",
    )
    assert left_out is not None and right_out is not None
    np.testing.assert_allclose(left_out, left)
    np.testing.assert_allclose(right_out, right)


def test_split_hand_input_single_right_fallback():
    right = np.arange(SINGLE_HAND_VALUES, dtype=np.float32)
    left_out, right_out = split_hand_input(
        right,
        enable_left_hand=True,
        enable_right_hand=True,
        single_hand_fallback_side="right",
    )
    assert left_out is None
    assert right_out is not None
    np.testing.assert_allclose(right_out.reshape(-1), right)
