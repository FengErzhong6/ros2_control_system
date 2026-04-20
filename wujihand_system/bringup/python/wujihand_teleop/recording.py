from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import threading
import time
import traceback
from typing import Any, Optional

import numpy as np


RAW_NODE_ID_FILL = -1
RAW_PARENT_ID_FILL = -1
HAND_INPUT_CAPACITY = 21 * 3 * 2


@dataclass(slots=True)
class RawHandFrame:
    glove_id: int
    node_ids: np.ndarray
    parent_ids: np.ndarray
    positions: np.ndarray
    orientations: np.ndarray


@dataclass(slots=True)
class AlignedRecordedFrame:
    frame_index: int
    raw_stamp_ns: int
    record_time_ns: int
    left_raw: Optional[RawHandFrame]
    right_raw: Optional[RawHandFrame]
    hand_input: Optional[np.ndarray]
    left_keypoints: Optional[np.ndarray]
    right_keypoints: Optional[np.ndarray]
    left_qpos: Optional[np.ndarray]
    right_qpos: Optional[np.ndarray]


def _utc_now_string() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")


def _stamp_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def raw_message_stamp_ns(msg: Any) -> int:
    return _stamp_to_ns(msg.header.stamp)


def _normalize_side(value: Any) -> str:
    return str(value).strip().lower()


def extract_raw_hands(msg: Any) -> tuple[Optional[RawHandFrame], Optional[RawHandFrame]]:
    left_raw: Optional[RawHandFrame] = None
    right_raw: Optional[RawHandFrame] = None

    for glove in getattr(msg, "gloves", []):
        node_ids = []
        parent_ids = []
        positions = []
        orientations = []
        for raw_node in getattr(glove, "raw_nodes", []):
            node_ids.append(int(raw_node.node_id))
            parent_ids.append(int(raw_node.parent_node_id))
            positions.append(
                [
                    float(raw_node.pose.position.x),
                    float(raw_node.pose.position.y),
                    float(raw_node.pose.position.z),
                ]
            )
            orientations.append(
                [
                    float(raw_node.pose.orientation.x),
                    float(raw_node.pose.orientation.y),
                    float(raw_node.pose.orientation.z),
                    float(raw_node.pose.orientation.w),
                ]
            )

        hand = RawHandFrame(
            glove_id=int(getattr(glove, "glove_id", -1)),
            node_ids=np.asarray(node_ids, dtype=np.int32),
            parent_ids=np.asarray(parent_ids, dtype=np.int32),
            positions=np.asarray(positions, dtype=np.float32).reshape(-1, 3),
            orientations=np.asarray(orientations, dtype=np.float32).reshape(-1, 4),
        )

        side = _normalize_side(getattr(glove, "side", ""))
        if side == "left":
            left_raw = hand
        elif side == "right":
            right_raw = hand

    return left_raw, right_raw


def build_aligned_record(
    *,
    frame_index: int,
    raw_msg: Any,
    hand_input: Optional[np.ndarray],
    left_keypoints: Optional[np.ndarray],
    right_keypoints: Optional[np.ndarray],
    left_qpos: Optional[np.ndarray],
    right_qpos: Optional[np.ndarray],
) -> AlignedRecordedFrame:
    left_raw, right_raw = extract_raw_hands(raw_msg)
    return AlignedRecordedFrame(
        frame_index=frame_index,
        raw_stamp_ns=raw_message_stamp_ns(raw_msg),
        record_time_ns=time.time_ns(),
        left_raw=left_raw,
        right_raw=right_raw,
        hand_input=None if hand_input is None else np.asarray(hand_input, dtype=np.float32).reshape(-1).copy(),
        left_keypoints=None if left_keypoints is None else np.asarray(left_keypoints, dtype=np.float32).reshape(21, 3).copy(),
        right_keypoints=None if right_keypoints is None else np.asarray(right_keypoints, dtype=np.float32).reshape(21, 3).copy(),
        left_qpos=None if left_qpos is None else np.asarray(left_qpos, dtype=np.float64).reshape(-1).copy(),
        right_qpos=None if right_qpos is None else np.asarray(right_qpos, dtype=np.float64).reshape(-1).copy(),
    )


class AlignedFrameRecorder:
    def __init__(
        self,
        *,
        output_root: str | Path,
        chunk_size: int,
        queue_size: int,
        flush_period_sec: float,
        logger: Any,
        config_snapshot: dict[str, Any],
    ) -> None:
        self._output_root = Path(output_root).expanduser()
        self._chunk_size = max(int(chunk_size), 1)
        self._flush_period_sec = max(float(flush_period_sec), 0.1)
        self._logger = logger
        self._config_snapshot = dict(config_snapshot)
        self._queue: queue.Queue[AlignedRecordedFrame] = queue.Queue(maxsize=max(int(queue_size), 1))
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._writer_loop, name="aligned-frame-recorder", daemon=True)
        self._chunk_index = 0
        self._frames_written = 0
        self._frames_dropped = 0
        self._written_chunks: list[dict[str, Any]] = []
        self._thread_error: Optional[str] = None
        session_name = f"session_{_utc_now_string()}_{os.getpid()}"
        self._session_dir = self._output_root / session_name
        self._session_dir.mkdir(parents=True, exist_ok=False)
        self._write_metadata(final=False)
        self._thread.start()
        self._logger.info(f"Aligned recorder enabled. output_dir={self._session_dir}")

    @property
    def session_dir(self) -> Path:
        return self._session_dir

    def record(self, frame: AlignedRecordedFrame) -> None:
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            self._frames_dropped += 1
            if self._frames_dropped == 1 or (self._frames_dropped % 100) == 0:
                self._logger.warning(
                    f"Aligned recorder queue full; dropped_frames={self._frames_dropped}"
                )

    def close(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=10.0)
        if self._thread.is_alive():
            self._logger.warning("Aligned recorder writer thread did not stop before timeout.")
        if self._thread_error is not None:
            self._logger.error("Aligned recorder writer thread failed:\n" + self._thread_error)
        self._write_metadata(final=True)

    def _writer_loop(self) -> None:
        chunk: list[AlignedRecordedFrame] = []
        try:
            while not self._stop_event.is_set() or not self._queue.empty():
                try:
                    frame = self._queue.get(timeout=self._flush_period_sec)
                    chunk.append(frame)
                    if len(chunk) >= self._chunk_size:
                        self._flush_chunk(chunk)
                        chunk = []
                except queue.Empty:
                    if chunk:
                        self._flush_chunk(chunk)
                        chunk = []
            if chunk:
                self._flush_chunk(chunk)
        except Exception:
            self._thread_error = traceback.format_exc()
            self._logger.error("Aligned recorder writer loop crashed:\n" + self._thread_error)

    def _flush_chunk(self, chunk: list[AlignedRecordedFrame]) -> None:
        arrays = self._chunk_to_arrays(chunk)
        file_name = f"aligned_chunk_{self._chunk_index:06d}.npz"
        target = self._session_dir / file_name
        tmp_target = target.with_suffix(".npz.tmp")
        with tmp_target.open("wb") as handle:
            np.savez(handle, **arrays)
        tmp_target.replace(target)
        self._written_chunks.append(
            {
                "file": file_name,
                "frame_count": len(chunk),
                "raw_stamp_ns_start": int(chunk[0].raw_stamp_ns),
                "raw_stamp_ns_end": int(chunk[-1].raw_stamp_ns),
            }
        )
        self._chunk_index += 1
        self._frames_written += len(chunk)

    def _chunk_to_arrays(self, chunk: list[AlignedRecordedFrame]) -> dict[str, np.ndarray]:
        n = len(chunk)
        left_max_nodes = max((frame.left_raw.node_ids.size for frame in chunk if frame.left_raw is not None), default=0)
        right_max_nodes = max((frame.right_raw.node_ids.size for frame in chunk if frame.right_raw is not None), default=0)

        data: dict[str, np.ndarray] = {
            "frame_index": np.asarray([frame.frame_index for frame in chunk], dtype=np.int64),
            "raw_stamp_ns": np.asarray([frame.raw_stamp_ns for frame in chunk], dtype=np.int64),
            "record_time_ns": np.asarray([frame.record_time_ns for frame in chunk], dtype=np.int64),
            "left_raw_present": np.asarray([frame.left_raw is not None for frame in chunk], dtype=np.bool_),
            "right_raw_present": np.asarray([frame.right_raw is not None for frame in chunk], dtype=np.bool_),
            "left_raw_glove_id": np.full((n,), -1, dtype=np.int64),
            "right_raw_glove_id": np.full((n,), -1, dtype=np.int64),
            "left_raw_node_count": np.zeros((n,), dtype=np.int32),
            "right_raw_node_count": np.zeros((n,), dtype=np.int32),
            "left_raw_node_ids": np.full((n, left_max_nodes), RAW_NODE_ID_FILL, dtype=np.int32),
            "right_raw_node_ids": np.full((n, right_max_nodes), RAW_NODE_ID_FILL, dtype=np.int32),
            "left_raw_parent_ids": np.full((n, left_max_nodes), RAW_PARENT_ID_FILL, dtype=np.int32),
            "right_raw_parent_ids": np.full((n, right_max_nodes), RAW_PARENT_ID_FILL, dtype=np.int32),
            "left_raw_positions": np.full((n, left_max_nodes, 3), np.nan, dtype=np.float32),
            "right_raw_positions": np.full((n, right_max_nodes, 3), np.nan, dtype=np.float32),
            "left_raw_orientations": np.full((n, left_max_nodes, 4), np.nan, dtype=np.float32),
            "right_raw_orientations": np.full((n, right_max_nodes, 4), np.nan, dtype=np.float32),
            "hand_input_valid_len": np.zeros((n,), dtype=np.int32),
            "hand_input_flat": np.full((n, HAND_INPUT_CAPACITY), np.nan, dtype=np.float32),
            "left_keypoints_present": np.asarray([frame.left_keypoints is not None for frame in chunk], dtype=np.bool_),
            "right_keypoints_present": np.asarray([frame.right_keypoints is not None for frame in chunk], dtype=np.bool_),
            "left_keypoints": np.full((n, 21, 3), np.nan, dtype=np.float32),
            "right_keypoints": np.full((n, 21, 3), np.nan, dtype=np.float32),
            "left_qpos_present": np.asarray([frame.left_qpos is not None for frame in chunk], dtype=np.bool_),
            "right_qpos_present": np.asarray([frame.right_qpos is not None for frame in chunk], dtype=np.bool_),
            "left_qpos": np.full((n, 20), np.nan, dtype=np.float64),
            "right_qpos": np.full((n, 20), np.nan, dtype=np.float64),
        }

        for i, frame in enumerate(chunk):
            if frame.left_raw is not None:
                count = frame.left_raw.node_ids.size
                data["left_raw_glove_id"][i] = frame.left_raw.glove_id
                data["left_raw_node_count"][i] = count
                data["left_raw_node_ids"][i, :count] = frame.left_raw.node_ids
                data["left_raw_parent_ids"][i, :count] = frame.left_raw.parent_ids
                data["left_raw_positions"][i, :count, :] = frame.left_raw.positions
                data["left_raw_orientations"][i, :count, :] = frame.left_raw.orientations
            if frame.right_raw is not None:
                count = frame.right_raw.node_ids.size
                data["right_raw_glove_id"][i] = frame.right_raw.glove_id
                data["right_raw_node_count"][i] = count
                data["right_raw_node_ids"][i, :count] = frame.right_raw.node_ids
                data["right_raw_parent_ids"][i, :count] = frame.right_raw.parent_ids
                data["right_raw_positions"][i, :count, :] = frame.right_raw.positions
                data["right_raw_orientations"][i, :count, :] = frame.right_raw.orientations
            if frame.hand_input is not None:
                valid_len = min(frame.hand_input.size, HAND_INPUT_CAPACITY)
                data["hand_input_valid_len"][i] = valid_len
                data["hand_input_flat"][i, :valid_len] = frame.hand_input[:valid_len]
            if frame.left_keypoints is not None:
                data["left_keypoints"][i] = frame.left_keypoints
            if frame.right_keypoints is not None:
                data["right_keypoints"][i] = frame.right_keypoints
            if frame.left_qpos is not None:
                data["left_qpos"][i, : frame.left_qpos.size] = frame.left_qpos
            if frame.right_qpos is not None:
                data["right_qpos"][i, : frame.right_qpos.size] = frame.right_qpos

        return data

    def _write_metadata(self, *, final: bool) -> None:
        metadata = {
            "schema_version": 1,
            "created_at_utc": _utc_now_string(),
            "finalized": final,
            "session_dir": str(self._session_dir),
            "chunk_size": self._chunk_size,
            "flush_period_sec": self._flush_period_sec,
            "hand_input_capacity": HAND_INPUT_CAPACITY,
            "frames_written": self._frames_written,
            "frames_dropped": self._frames_dropped,
            "chunks": list(self._written_chunks),
            "config": self._config_snapshot,
        }
        metadata_path = self._session_dir / "metadata.json"
        metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")
