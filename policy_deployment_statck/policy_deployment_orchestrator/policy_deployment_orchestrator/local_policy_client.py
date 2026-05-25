from __future__ import annotations

import logging
import os
import sys
import time
from typing import Any

logger = logging.getLogger(__name__)


class LocalPolicyClient:
    """Loads and runs an OpenPI policy in-process for zero-copy, zero-latency inference.

    This replaces the WebSocket round-trip with a direct Python call. The first
    ``infer()`` triggers JAX JIT compilation, which may take 10-30 seconds; all
    subsequent calls run at full speed.
    """

    def __init__(
        self,
        *,
        config_name: str,
        checkpoint_dir: str,
        openpi_src_path: str = "",
        openpi_client_src_path: str = "",
        default_prompt: str | None = None,
    ) -> None:
        self._config_name = config_name
        self._checkpoint_dir = checkpoint_dir
        self._openpi_src_path = openpi_src_path
        self._openpi_client_src_path = openpi_client_src_path
        self._default_prompt = default_prompt
        self._policy: Any = None
        self._metadata: dict[str, Any] = {}
        self._infer_count = 0

    @property
    def metadata(self) -> dict[str, Any]:
        return dict(self._metadata)

    def connect(self) -> None:
        if self._policy is not None:
            return

        _inject_openpi_paths(self._openpi_src_path, self._openpi_client_src_path)
        _preimport_accelerate()

        try:
            from openpi.policies import policy_config
            from openpi.training import config as train_config
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                f"openpi is not importable. Install openpi or set local_policy_openpi_src. "
                f"Details: {exc}"
            ) from exc

        logger.info(
            "Loading local policy config=%s checkpoint=%s",
            self._config_name,
            self._checkpoint_dir,
        )
        t_start = time.monotonic()
        policy = policy_config.create_trained_policy(
            train_config.get_config(self._config_name),
            self._checkpoint_dir,
            default_prompt=self._default_prompt,
        )
        load_ms = (time.monotonic() - t_start) * 1000.0
        logger.info(
            "Local policy loaded in %.0f ms (first infer will JIT-compile).",
            load_ms,
        )
        self._policy = policy
        self._metadata = policy.metadata

    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        if self._policy is None:
            self.connect()
        assert self._policy is not None

        if self._infer_count == 0:
            logger.info("First inference — JAX JIT compilation in progress, may take 10-30 s ...")
            t_start = time.monotonic()

        result = self._policy.infer(observation)

        if self._infer_count == 0:
            compile_ms = (time.monotonic() - t_start) * 1000.0
            logger.info("JIT compilation complete in %.0f ms.", compile_ms)

        self._infer_count += 1
        return result

    def close(self) -> None:
        self._policy = None
        self._metadata = {}


def _inject_openpi_paths(openpi_src: str, openpi_client_src: str) -> None:
    # When spawned by ros2 launch the child process runs /usr/bin/python3
    # (the shebang of the generated console_script entry point) even though
    # the parent was launched from the venv.  Recover the venv's site-packages
    # so that JAX and the rest of the openpi dependency tree are importable.
    venv = os.environ.get("VIRTUAL_ENV")
    if venv:
        py_ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
        site_packages = os.path.join(venv, "lib", py_ver, "site-packages")
        if os.path.isdir(site_packages) and site_packages not in sys.path:
            sys.path.insert(0, site_packages)
        elif not os.path.isdir(site_packages):
            lib_dir = os.path.join(venv, "lib")
            candidates = []
            if os.path.isdir(lib_dir):
                candidates = [
                    entry
                    for entry in sorted(os.listdir(lib_dir))
                    if entry.startswith("python")
                    and os.path.isdir(os.path.join(lib_dir, entry, "site-packages"))
                ]
            if candidates:
                raise RuntimeError(
                    f"VIRTUAL_ENV points to {venv}, but the supervisor is running {py_ver}. "
                    f"Found site-packages for {', '.join(candidates)} instead. "
                    f"Recreate the policy venv with {py_ver} for in-process local policy, "
                    "or run OpenPI in a separate Python process."
                )

    for path in (openpi_src, openpi_client_src):
        if path and path not in sys.path:
            sys.path.insert(0, path)


def _preimport_accelerate() -> None:
    # Import after venv path injection so transformers sees a fully initialised
    # accelerate module during the OpenPI import chain.
    try:
        import accelerate  # noqa: F401
    except ModuleNotFoundError as exc:
        if exc.name == "accelerate":
            return
        raise
