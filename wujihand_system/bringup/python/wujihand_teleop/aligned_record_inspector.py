from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


LANDMARK_NAMES = [
    "WRIST",
    "THUMB_CMC",
    "THUMB_MCP",
    "THUMB_IP",
    "THUMB_TIP",
    "INDEX_MCP",
    "INDEX_PIP",
    "INDEX_DIP",
    "INDEX_TIP",
    "MIDDLE_MCP",
    "MIDDLE_PIP",
    "MIDDLE_DIP",
    "MIDDLE_TIP",
    "RING_MCP",
    "RING_PIP",
    "RING_DIP",
    "RING_TIP",
    "PINKY_MCP",
    "PINKY_PIP",
    "PINKY_DIP",
    "PINKY_TIP",
]

JOINT_NAMES = [
    "finger1_joint1",
    "finger1_joint2",
    "finger1_joint3",
    "finger1_joint4",
    "finger2_joint1",
    "finger2_joint2",
    "finger2_joint3",
    "finger2_joint4",
    "finger3_joint1",
    "finger3_joint2",
    "finger3_joint3",
    "finger3_joint4",
    "finger4_joint1",
    "finger4_joint2",
    "finger4_joint3",
    "finger4_joint4",
    "finger5_joint1",
    "finger5_joint2",
    "finger5_joint3",
    "finger5_joint4",
]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect aligned Manus recording chunks and print readable tables."
    )
    parser.add_argument(
        "--root",
        default="/tmp/wujihand_manus_records",
        help="Recorder output root directory.",
    )
    parser.add_argument(
        "--session",
        default="",
        help="Session directory name or absolute path. Default: latest session.",
    )
    parser.add_argument(
        "--chunk",
        type=int,
        default=-1,
        help="Chunk index to inspect. Default: latest chunk.",
    )
    parser.add_argument(
        "--frame-index",
        type=int,
        default=-1,
        help="Global frame_index to inspect inside the selected chunk. Default: latest frame in chunk.",
    )
    parser.add_argument(
        "--row",
        type=int,
        default=-1,
        help="Row offset inside the selected chunk. Overrides --frame-index if set.",
    )
    parser.add_argument(
        "--show-hand-input-flat",
        action="store_true",
        help="Also print flattened /hand_input values grouped by hand.",
    )
    return parser.parse_args()


def _latest_entry(entries: Iterable[Path]) -> Path:
    ordered = sorted(entries, key=lambda path: (path.stat().st_mtime_ns, path.name))
    if not ordered:
        raise FileNotFoundError("No matching entries found.")
    return ordered[-1]


def _resolve_session(root: Path, session_arg: str) -> Path:
    if session_arg:
        candidate = Path(session_arg).expanduser()
        if candidate.is_dir():
            return candidate.resolve()
        candidate = root / session_arg
        if candidate.is_dir():
            return candidate.resolve()
        raise FileNotFoundError(f"Session directory not found: {session_arg}")
    return _latest_entry(root.glob("session_*")).resolve()


def _resolve_chunk(session_dir: Path, chunk_index: int) -> Path:
    if chunk_index >= 0:
        candidate = session_dir / f"aligned_chunk_{chunk_index:06d}.npz"
        if not candidate.is_file():
            raise FileNotFoundError(f"Chunk file not found: {candidate}")
        return candidate
    return _latest_entry(session_dir.glob("aligned_chunk_*.npz"))


def _resolve_row(data: np.lib.npyio.NpzFile, row: int, frame_index: int) -> int:
    frame_indices = data["frame_index"]
    if frame_indices.size == 0:
        raise ValueError("Selected chunk contains no frames.")
    if row >= 0:
        if row >= frame_indices.shape[0]:
            raise IndexError(f"Row {row} out of range for chunk of size {frame_indices.shape[0]}.")
        return row
    if frame_index >= 0:
        matches = np.where(frame_indices == frame_index)[0]
        if matches.size == 0:
            raise ValueError(f"frame_index={frame_index} not found in selected chunk.")
        return int(matches[0])
    return int(frame_indices.shape[0] - 1)


def _fmt_float(value: float) -> str:
    if np.isnan(value):
        return "nan"
    return f"{value:.6f}"


def _format_table(headers: Sequence[str], rows: Sequence[Sequence[str]]) -> str:
    widths = [len(header) for header in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(cell))
    def fmt_row(values: Sequence[str]) -> str:
        return " | ".join(value.ljust(widths[i]) for i, value in enumerate(values))
    separator = "-+-".join("-" * width for width in widths)
    lines = [fmt_row(headers), separator]
    lines.extend(fmt_row(row) for row in rows)
    return "\n".join(lines)


def _raw_rows(prefix: str, record: np.lib.npyio.NpzFile, row: int) -> list[list[str]]:
    node_count = int(record[f"{prefix}_raw_node_count"][row])
    if node_count <= 0:
        return []
    node_ids = record[f"{prefix}_raw_node_ids"][row, :node_count]
    parent_ids = record[f"{prefix}_raw_parent_ids"][row, :node_count]
    positions = record[f"{prefix}_raw_positions"][row, :node_count]
    orientations = record[f"{prefix}_raw_orientations"][row, :node_count]
    rows: list[list[str]] = []
    for i in range(node_count):
        rows.append(
            [
                str(i),
                str(int(node_ids[i])),
                str(int(parent_ids[i])),
                _fmt_float(float(positions[i, 0])),
                _fmt_float(float(positions[i, 1])),
                _fmt_float(float(positions[i, 2])),
                _fmt_float(float(orientations[i, 0])),
                _fmt_float(float(orientations[i, 1])),
                _fmt_float(float(orientations[i, 2])),
                _fmt_float(float(orientations[i, 3])),
            ]
        )
    return rows


def _keypoint_rows(prefix: str, record: np.lib.npyio.NpzFile, row: int) -> list[list[str]]:
    present = bool(record[f"{prefix}_keypoints_present"][row])
    if not present:
        return []
    keypoints = record[f"{prefix}_keypoints"][row]
    return [
        [
            str(i),
            LANDMARK_NAMES[i],
            _fmt_float(float(keypoints[i, 0])),
            _fmt_float(float(keypoints[i, 1])),
            _fmt_float(float(keypoints[i, 2])),
        ]
        for i in range(keypoints.shape[0])
    ]


def _qpos_rows(prefix: str, record: np.lib.npyio.NpzFile, row: int) -> list[list[str]]:
    present = bool(record[f"{prefix}_qpos_present"][row])
    if not present:
        return []
    qpos = record[f"{prefix}_qpos"][row]
    return [
        [str(i), JOINT_NAMES[i], _fmt_float(float(qpos[i]))]
        for i in range(qpos.shape[0])
    ]


def _print_hand_input_flat(record: np.lib.npyio.NpzFile, row: int) -> None:
    valid_len = int(record["hand_input_valid_len"][row])
    if valid_len <= 0:
        print("\nHand Input Flat: not present")
        return
    flat = record["hand_input_flat"][row, :valid_len]
    print(f"\nHand Input Flat: valid_len={valid_len}")
    for start in range(0, valid_len, 9):
        values = " ".join(f"{float(value): .6f}" for value in flat[start:start + 9])
        print(f"[{start:03d}:{min(start + 9, valid_len):03d}] {values}")


def main() -> None:
    args = _parse_args()
    root = Path(args.root).expanduser().resolve()
    session_dir = _resolve_session(root, args.session)
    chunk_path = _resolve_chunk(session_dir, args.chunk)
    metadata_path = session_dir / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.is_file() else {}

    with np.load(chunk_path) as record:
        row = _resolve_row(record, args.row, args.frame_index)
        frame_index = int(record["frame_index"][row])
        raw_stamp_ns = int(record["raw_stamp_ns"][row])
        print(f"Session: {session_dir}")
        print(f"Chunk:   {chunk_path.name}")
        print(f"Row:     {row}")
        print(f"Frame:   {frame_index}")
        print(f"Raw Ns:  {raw_stamp_ns}")
        if metadata:
            print(
                f"Written: {metadata.get('frames_written', 'unknown')} frames, "
                f"Dropped: {metadata.get('frames_dropped', 'unknown')} frames"
            )

        for prefix, title in (("left", "Left"), ("right", "Right")):
            glove_id = int(record[f"{prefix}_raw_glove_id"][row])
            raw_rows = _raw_rows(prefix, record, row)
            print(f"\n{title} Raw Nodes: glove_id={glove_id if glove_id >= 0 else 'N/A'}")
            if raw_rows:
                print(
                    _format_table(
                        ["idx", "node_id", "parent_id", "px", "py", "pz", "qx", "qy", "qz", "qw"],
                        raw_rows,
                    )
                )
            else:
                print("not present")

            kp_rows = _keypoint_rows(prefix, record, row)
            print(f"\n{title} Retarget Input (21x3):")
            if kp_rows:
                print(_format_table(["idx", "landmark", "x", "y", "z"], kp_rows))
            else:
                print("not present")

            qpos_rows = _qpos_rows(prefix, record, row)
            print(f"\n{title} Target Qpos (20):")
            if qpos_rows:
                print(_format_table(["idx", "joint", "qpos"], qpos_rows))
            else:
                print("not present")

        if args.show_hand_input_flat:
            _print_hand_input_flat(record, row)


if __name__ == "__main__":
    main()
