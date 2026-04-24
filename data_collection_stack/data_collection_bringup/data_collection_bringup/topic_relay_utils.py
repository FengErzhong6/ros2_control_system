from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class TopicRelaySpec:
    source_topic: str
    output_topic: str
    msg_type: Any
    output_msg_type: Any | None = None
    rewrite_header_stamp: bool = True


def normalize_topic(topic: str) -> str:
    value = str(topic).strip()
    if not value:
        raise RuntimeError("Topic must not be empty.")
    if not value.startswith("/"):
        raise RuntimeError(f"Topic must be absolute: {value}")
    return value


def relay_topic_name(source_topic: str, prefix: str) -> str:
    normalized_source = normalize_topic(source_topic)
    normalized_prefix = normalize_topic(prefix).rstrip("/")
    return f"{normalized_prefix}{normalized_source}"


def clone_message_with_stamp(message: Any, stamp: Any) -> Any:
    cloned = deepcopy(message)
    header = getattr(cloned, "header", None)
    if header is None:
        raise RuntimeError(f"Message type {type(cloned).__name__} has no header field.")
    header.stamp = stamp
    return cloned
