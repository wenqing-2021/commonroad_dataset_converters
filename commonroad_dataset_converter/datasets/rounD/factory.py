from dataclasses import dataclass
from pathlib import Path

from commonroad_dataset_converter.conversion.tabular.job_consumer import (
    TabularJobConsumer,
)
from commonroad_dataset_converter.datasets.common.levelx_datasets import (
    LevelXConverterFactory,
)


class RounDConverterFactory(LevelXConverterFactory):
    def get_config_path(self) -> Path:
        return Path(__file__).parent / "config.yaml"

    def build_job_processor(self) -> TabularJobConsumer:
        proc = super().build_job_processor()
        proc.create_turning_indicator = True
        return proc
