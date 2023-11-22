from abc import ABCMeta
from dataclasses import field
from pathlib import Path
from typing import Any, Mapping

from pydantic import PrivateAttr

from commonroad_dataset_converter.conversion.tabular.factory import (
    TabularConverterFactory,
)
from commonroad_dataset_converter.conversion.util.yaml import load_yaml

from .implementation import (
    InteractionMetaScenarioGenerator,
    InteractionRecording,
    InteractionRecordingGenerator,
)


class InteractionConverterFactory(
    TabularConverterFactory[InteractionRecording], metaclass=ABCMeta
):
    input_dir: Path
    _config: Mapping[str, Any] = PrivateAttr(
        default_factory=lambda: load_yaml(str(Path(__file__).parent / "config.yaml"))
    )

    def build_recording_generator(self) -> InteractionRecordingGenerator:
        return InteractionRecordingGenerator(
            self.input_dir,
            self._config["directory_data"],
            {
                location: (offsets["x_offset_tracks"], offsets["y_offset_tracks"])
                for location, offsets in self._config["offsets"].items()
            },
        )

    def build_meta_scenario_creator(self) -> InteractionMetaScenarioGenerator:
        return InteractionMetaScenarioGenerator(self._config["maps"])
