#!/usr/bin/env python3
"""Standalone local policy warmup test — no hardware required.

Usage:
    source .venv_policy/bin/activate
    python scripts/warmup_test.py
"""

import os
import sys
import time

# Inject venv site-packages (in case the python is not the venv python).
venv = os.environ.get("VIRTUAL_ENV")
if venv:
    py_ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    site_pkg = os.path.join(venv, "lib", py_ver, "site-packages")
    if os.path.isdir(site_pkg) and site_pkg not in sys.path:
        sys.path.insert(0, site_pkg)

# Inject OpenPI source paths.
_OPENPI_SRC = "/home/mmlab/codes/huangshzh/openpi_dexhand/src"
_OPENPI_CLIENT_SRC = "/home/mmlab/codes/huangshzh/openpi_dexhand/packages/openpi-client/src"
for p in (_OPENPI_SRC, _OPENPI_CLIENT_SRC):
    if p not in sys.path:
        sys.path.insert(0, p)

# Pre-import accelerate so transformers sees it.
import accelerate  # noqa: E402, F401

import numpy as np  # noqa: E402
from openpi.policies import policy_config  # noqa: E402
from openpi.training import config as train_config  # noqa: E402


def main():
    config_name = "pi05_tianji_wuji_pick_place"
    checkpoint_dir = (
        "/home/mmlab/codes/huangshzh/openpi_dexhand/checkpoints/"
        "pi05_tianji_wuji_pick_place/hsz_right_pick_place/29999"
    )
    default_prompt = "pick and place."

    print(f"Loading policy config: {config_name}")
    t0 = time.monotonic()
    policy = policy_config.create_trained_policy(
        train_config.get_config(config_name),
        checkpoint_dir,
        default_prompt=default_prompt,
    )
    load_ms = (time.monotonic() - t0) * 1000
    print(f"Model loaded in {load_ms:.0f} ms")
    print(f"Metadata: {policy.metadata}")

    # Build synthetic observation matching the profile spec:
    #   images: cam_high (224,224,3), cam_left_wrist (224,224,3)
    #   state: 27D (right_arm 7 + right_hand 20)
    obs = {
        "images": {
            "cam_high": np.random.rand(224, 224, 3).astype(np.float32),
            "cam_left_wrist": np.random.rand(224, 224, 3).astype(np.float32),
        },
        "state": np.random.randn(27).astype(np.float32),
        "prompt": default_prompt,
    }
    print("Observation shapes:")
    for k, v in obs["images"].items():
        print(f"  images/{k}: {v.shape} {v.dtype}")
    print(f"  state: {obs['state'].shape} {obs['state'].dtype}")
    print(f"  prompt: {obs['prompt']}")

    print("\nRunning first inference (JAX JIT compilation, may take 10-30s) ...")
    t0 = time.monotonic()
    result = policy.infer(obs)
    compile_ms = (time.monotonic() - t0) * 1000
    print(f"JIT warmup complete in {compile_ms:.0f} ms")

    actions = result.get("actions")
    if actions is not None:
        print(f"Output actions shape: {np.asarray(actions).shape}")

    # Run a second inference to confirm it's fast.
    print("\nRunning second inference (should be fast) ...")
    t0 = time.monotonic()
    policy.infer(obs)
    second_ms = (time.monotonic() - t0) * 1000
    print(f"Second inference: {second_ms:.0f} ms")

    print("\nWarmup test PASSED.")


if __name__ == "__main__":
    main()
