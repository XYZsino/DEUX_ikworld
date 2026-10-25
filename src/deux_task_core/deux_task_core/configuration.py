"""Load a static installation snapshot independently in each consuming node."""

from pathlib import Path
import yaml
from .model import RobotModel
from .planning import validate_scene, scene_version


def load_configuration(filename):
    if not filename:
        raise ValueError("config_file must name an installation YAML file")
    path = Path(filename).expanduser().resolve()
    config = yaml.safe_load(path.read_text())
    if not isinstance(config, dict):
        raise ValueError("installation YAML must contain a mapping")
    validate_scene(config)
    model_path = Path(config["robot"]["urdf_path"]).expanduser()
    if not model_path.is_absolute():
        model_path = path.parent / model_path
    config["robot"]["urdf_path"] = str(model_path.resolve())
    version = scene_version(config)
    RobotModel(config["robot"])
    if scene_version(config) != version:
        raise ValueError("URDF changed while loading configuration")
    return config, version
