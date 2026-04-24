from __future__ import annotations


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
