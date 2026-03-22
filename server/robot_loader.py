from __future__ import annotations

import importlib
from pathlib import Path
from typing import Type

from .robot_base import RobotBase


class RobotLoadError(RuntimeError):
    """Configuration or import error while loading a robot class."""


def read_robot_spec(conf_path: str | Path) -> str:
    path = Path(conf_path)
    if not path.exists():
        raise RobotLoadError(f"robot config not found: {path}")
    raw = path.read_text(encoding="utf-8").strip()
    if not raw:
        raise RobotLoadError(f"robot config is empty: {path}")
    return raw


def parse_robot_spec(spec: str) -> tuple[str, str]:
    if ":" not in spec:
        raise RobotLoadError(
            f"invalid robot spec '{spec}': expected 'module.path:ClassName'"
        )
    module_path, class_name = spec.split(":", 1)
    module_path = module_path.strip()
    class_name = class_name.strip()
    if not module_path or not class_name:
        raise RobotLoadError(
            f"invalid robot spec '{spec}': expected 'module.path:ClassName'"
        )
    return module_path, class_name


def load_robot_class(conf_path: str | Path) -> Type[RobotBase]:
    spec = read_robot_spec(conf_path)
    module_path, class_name = parse_robot_spec(spec)

    try:
        mod = importlib.import_module(module_path)
    except Exception as exc:
        raise RobotLoadError(
            f"failed to import module '{module_path}' from '{conf_path}': {exc}"
        ) from exc

    try:
        cls = getattr(mod, class_name)
    except AttributeError as exc:
        raise RobotLoadError(
            f"class '{class_name}' not found in module '{module_path}'"
        ) from exc

    if not isinstance(cls, type):
        raise RobotLoadError(
            f"loaded object '{module_path}:{class_name}' is not a class"
        )

    if not issubclass(cls, RobotBase):
        raise RobotLoadError(
            f"class '{module_path}:{class_name}' does not inherit RobotBase"
        )

    return cls
