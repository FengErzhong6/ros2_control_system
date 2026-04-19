from __future__ import annotations

from collections import defaultdict, deque
from contextlib import contextmanager
import time
from typing import DefaultDict, Deque, Iterator

import numpy as np


class PeriodicProfiler:
    def __init__(self, enabled: bool, log_period_sec: float, logger) -> None:
        self.enabled = enabled
        self.log_period_sec = max(float(log_period_sec), 0.1)
        self._logger = logger
        self._samples: DefaultDict[str, Deque[float]] = defaultdict(lambda: deque(maxlen=512))
        self._last_log_monotonic = time.monotonic()

    @contextmanager
    def measure(self, name: str) -> Iterator[None]:
        if not self.enabled:
            yield
            return
        start = time.perf_counter()
        try:
            yield
        finally:
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            self._samples[name].append(elapsed_ms)

    def record(self, name: str, elapsed_ms: float) -> None:
        if not self.enabled:
            return
        self._samples[name].append(float(elapsed_ms))

    def maybe_log(self) -> None:
        if not self.enabled:
            return
        now = time.monotonic()
        if (now - self._last_log_monotonic) < self.log_period_sec:
            return

        lines = ["Teleop profiling (ms):"]
        for name in sorted(self._samples.keys()):
            samples = self._samples[name]
            if not samples:
                continue
            values = np.asarray(samples, dtype=np.float64)
            lines.append(
                f"{name}: avg={np.mean(values):.3f}, p95={np.percentile(values, 95):.3f}, "
                f"max={np.max(values):.3f}, n={values.size}"
            )

        if len(lines) > 1:
            self._logger.info(" | ".join(lines))
        self._last_log_monotonic = now
