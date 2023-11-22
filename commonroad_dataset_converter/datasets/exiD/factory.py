from pathlib import Path

from commonroad_dataset_converter.datasets.common.levelx_datasets import (
    LevelXConverterFactory,
)


class ExiDConverterFactory(LevelXConverterFactory):
    def get_config_path(self) -> Path:
        return Path(__file__).parent / "config.yaml"
