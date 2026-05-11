from __future__ import annotations

from pathlib import Path

import yaml

from .models import LaunchSpec, RecipeSpec


def discover_recipes(recipe_directory: Path | None) -> dict[str, Path]:
    if recipe_directory is None or not recipe_directory.exists():
        return {}

    recipes: dict[str, Path] = {}
    for candidate in sorted(recipe_directory.glob("*.yaml")):
        recipe_id = candidate.stem
        try:
            with candidate.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            if isinstance(data, dict) and data.get("recipe_id"):
                recipe_id = str(data["recipe_id"])
        except yaml.YAMLError:
            recipe_id = candidate.stem
        recipes[recipe_id] = candidate
    return recipes


def load_recipe(recipe_directory: Path | None, recipe_id: str) -> RecipeSpec | None:
    recipe_path = discover_recipes(recipe_directory).get(recipe_id)
    if recipe_path is None:
        return None
    return parse_recipe_file(recipe_path)


def parse_recipe_file(path: Path) -> RecipeSpec:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    if not isinstance(data, dict):
        raise RuntimeError(f"Expected recipe mapping in {path}")

    raw_devices = data.get("devices")
    if raw_devices is None:
        raw_devices = data.get("launches", [])
    if not isinstance(raw_devices, list):
        raise RuntimeError(f"Expected devices list in {path}")

    launches: list[LaunchSpec] = []
    for raw_device in raw_devices:
        if not isinstance(raw_device, dict):
            raise RuntimeError(f"Expected device mapping in {path}")
        device_id = str(raw_device.get("id", "")).strip()
        if not device_id:
            raise RuntimeError(f"Device entry in {path} is missing id")

        adapter = str(raw_device.get("adapter", "launch")).strip() or "launch"
        package = str(raw_device.get("package", "")).strip()
        launch_file = str(raw_device.get("launch_file", "")).strip()
        if not package or not launch_file:
            raise RuntimeError(
                f"Device entry {device_id!r} in {path} is missing package/launch_file"
            )

        arguments = raw_device.get("launch_arguments", raw_device.get("arguments", {})) or {}
        if not isinstance(arguments, dict):
            raise RuntimeError(f"Device entry {device_id!r} arguments must be a mapping")

        depends_on_raw = raw_device.get("depends_on", []) or []
        if not isinstance(depends_on_raw, list):
            raise RuntimeError(f"Device entry {device_id!r} depends_on must be a list")
        depends_on = tuple(
            item.strip() for item in (str(value) for value in depends_on_raw) if item.strip()
        )

        ready_controllers_raw = raw_device.get("ready_controllers", {}) or {}
        if not isinstance(ready_controllers_raw, dict):
            raise RuntimeError(
                f"Device entry {device_id!r} ready_controllers must be a mapping"
            )
        ready_controllers: dict[str, tuple[str, ...]] = {}
        for controller_name, raw_states in ready_controllers_raw.items():
            name = str(controller_name).strip()
            if not name:
                continue
            if isinstance(raw_states, str):
                states = tuple(
                    item.strip()
                    for item in raw_states.replace(",", " ").split()
                    if item.strip()
                )
            elif isinstance(raw_states, list):
                states = tuple(str(item).strip() for item in raw_states if str(item).strip())
            else:
                states = (str(raw_states).strip(),)
            ready_controllers[name] = states or ("active",)

        launches.append(
            LaunchSpec(
                device_id=device_id,
                adapter=adapter,
                package=package,
                launch_file=launch_file,
                arguments=dict(arguments),
                required=bool(raw_device.get("required", True)),
                depends_on=depends_on,
                startup_timeout_sec=float(raw_device.get("startup_timeout_sec", 30.0)),
                ready_stability_window_sec=float(raw_device.get("ready_stability_window_sec", 0.0)),
                ready_relaunch_attempts=max(
                    0,
                    int(raw_device.get("ready_relaunch_attempts", 0)),
                ),
                ready_relaunch_backoff_sec=max(
                    0.0,
                    float(raw_device.get("ready_relaunch_backoff_sec", 1.0)),
                ),
                ready_topics=tuple(str(item) for item in raw_device.get("ready_topics", [])),
                ready_services=tuple(str(item) for item in raw_device.get("ready_services", [])),
                ready_controllers=ready_controllers,
                ready_motion_status=bool(raw_device.get("ready_motion_status", False)),
                ready_motion_status_timeout_sec=max(
                    1.0,
                    float(raw_device.get("ready_motion_status_timeout_sec", 12.0)),
                ),
                summary=str(raw_device.get("summary", "")),
            )
        )

    return RecipeSpec(
        recipe_id=str(data.get("recipe_id", path.stem)),
        path=path,
        launches=tuple(launches),
        ready_timeout_sec=float(data.get("ready_timeout_sec", 45.0)),
        summary=str(data.get("summary", "")),
    )
