#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node


def _node_fqn(namespace: str, name: str) -> str:
    ns = (namespace or "").strip()
    if not ns or ns == "/":
        return f"/{name}"
    return f"{ns.rstrip('/')}/{name}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--poll-sec", type=float, default=0.2)
    parser.add_argument("--stable-count", type=int, default=3)
    parser.add_argument("--prefix", action="append", default=[])
    args, _unknown = parser.parse_known_args()

    prefixes = [prefix.strip() for prefix in args.prefix if prefix.strip()]
    if not prefixes:
        return 0

    rclpy.init(args=None)
    node = Node("ros_graph_waiter")
    try:
        deadline = time.monotonic() + max(0.0, args.timeout_sec)
        stable = 0
        while time.monotonic() < deadline and rclpy.ok():
            discovered = [
                _node_fqn(namespace, name)
                for name, namespace in node.get_node_names_and_namespaces()
            ]
            remaining = [
                name
                for name in discovered
                if any(name == prefix or name.startswith(prefix) for prefix in prefixes)
            ]
            if not remaining:
                stable += 1
                if stable >= max(1, args.stable_count):
                    return 0
            else:
                stable = 0
            time.sleep(max(0.05, args.poll_sec))

        discovered = [
            _node_fqn(namespace, name)
            for name, namespace in node.get_node_names_and_namespaces()
        ]
        remaining = [
            name
            for name in discovered
            if any(name == prefix or name.startswith(prefix) for prefix in prefixes)
        ]
        if remaining:
            print(
                "[ros_graph_waiter] Timed out waiting for nodes to disappear: "
                + ", ".join(sorted(remaining)),
                file=sys.stderr,
                flush=True,
            )
            return 1
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
