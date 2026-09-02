import yaml
from pathlib import Path

CONFIG_PATH = Path(__file__).resolve().parent.parent / "src" / "config.yaml"

with open(CONFIG_PATH) as f:
    config = yaml.safe_load(f)
