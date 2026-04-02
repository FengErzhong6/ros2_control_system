from __future__ import annotations

from pathlib import Path

import yaml

from .models import DeviceSpec, RecipeSpec


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
    recipes = discover_recipes(recipe_directory)
    recipe_path = recipes.get(recipe_id)
    if recipe_path is None:
        return None
    return parse_recipe_file(recipe_path)


def parse_recipe_file(path: Path) -> RecipeSpec:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    if not isinstance(data, dict):
        raise RuntimeError(f"Expected recipe mapping in {path}")

    devices: list[DeviceSpec] = []
    for raw_device in data.get("devices", []):
        if not isinstance(raw_device, dict):
            raise RuntimeError(f"Expected device mapping in {path}")
        device_id = str(raw_device["id"])
        adapter = str(raw_device["adapter"])
        required = bool(raw_device.get("required", True))
        depends_on = [str(item) for item in raw_device.get("depends_on", [])]
        config = {
            key: value
            for key, value in raw_device.items()
            if key not in {"id", "adapter", "required", "depends_on"}
        }
        devices.append(
            DeviceSpec(
                device_id=device_id,
                adapter=adapter,
                required=required,
                depends_on=depends_on,
                config=config,
            )
        )

    return RecipeSpec(
        recipe_id=str(data.get("recipe_id", path.stem)),
        path=path,
        devices=devices,
        record_topics=[str(topic) for topic in data.get("record_topics", [])],
    )
