from pathlib import Path
from typing import Any, Mapping

from pydantic import PrivateAttr

from commonroad_dataset_converter.conversion.tabular.factory import (
    TabularConverterFactory,
)
from commonroad_dataset_converter.conversion.util.yaml import load_yaml

from ..common.levelx_datasets import LevelXRecording
from .implementation import (
    HigdTabularJobProcessor,
    HighdMetaScenarioGenerator,
    HighdObstacleWindowGenerator,
    HighdRecordingGenerator,
)


class HighdConverterFactory(TabularConverterFactory[LevelXRecording]):
    input_dir: Path
    num_vertices: int = 10
    shoulder: bool = False
    keep_direction: bool = False
    lane_change: bool = False
    extend_width: float = 0.0
    _highd_config: Mapping[str, Any] = PrivateAttr(
        default_factory=lambda: load_yaml(str(Path(__file__).parent / "config.yaml"))
    )

    def build_recording_generator(self) -> HighdRecordingGenerator:
        return HighdRecordingGenerator(
            self.input_dir,
            self._highd_config["locations"],
        )

    def build_meta_scenario_creator(self) -> HighdMetaScenarioGenerator:
        lanelet_network_creator = HighdMetaScenarioGenerator(
            self.num_vertices,
            self._highd_config["road_length"],
            self._highd_config["road_offset"],
            extend_width=self.extend_width,
        )
        return lanelet_network_creator

    def build_job_processor(self) -> HigdTabularJobProcessor:
        return HigdTabularJobProcessor(
            self.output_dir,
            self.file_writer_type,
            self.obstacles_start_at_zero,
            self.keep_direction,
        )

    def build_obstacle_window_generator(self):
        return HighdObstacleWindowGenerator(self.lane_change)
