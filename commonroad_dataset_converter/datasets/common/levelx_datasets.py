import copy
from abc import ABCMeta, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Tuple

import numpy as np
import pandas as pd
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.scenario import Scenario
from pydantic import PrivateAttr

from commonroad_dataset_converter.conversion.tabular.factory import (
    TabularConverterFactory,
)
from commonroad_dataset_converter.conversion.tabular.interface import (
    IMetaScenarioCreator,
    IRecordingGenerator,
    Window,
)
from commonroad_dataset_converter.conversion.util.yaml import load_yaml
from commonroad_dataset_converter.datasets.common.obstacle_utils import get_dt


@dataclass
class LevelXRecording:
    recording_meta: Dict[str, Any]


@dataclass
class LevelXMetaScenarioGenerator(IMetaScenarioCreator[LevelXRecording]):
    map_dir: Path
    map_names: Dict[int, str]
    _name_to_map: Dict[int, Scenario] = field(default_factory=dict, init=False)

    def __post_init__(self) -> None:
        for map_id, name in self.map_names.items():
            scenario = CommonRoadFileReader(str(self.map_dir / (name + ".xml"))).open()[
                0
            ]
            self._name_to_map[map_id] = scenario

    def __call__(self, window: Window, window_meta: LevelXRecording) -> Scenario:
        # TODO: Resolve typing issue
        meta_scenario = copy.deepcopy(
            self._name_to_map[window_meta.recording_meta["locationId"]]
        )
        meta_scenario.scenario_id.configuration_id = window_meta.recording_meta[
            "recordingId"
        ]
        return meta_scenario


@dataclass
class LevelXRecordingGenerator(IRecordingGenerator[LevelXRecording]):
    data_path: Path
    locations: Dict[int, str]
    obstacle_types: Dict[str, str]

    def __iter__(self) -> Iterable[Tuple[Window, LevelXRecording]]:
        recordings = list(self.data_path.rglob("*_tracks.csv"))
        assert self.data_path.exists()
        assert len(recordings) > 0
        for tracks_path in recordings:
            tracks_meta_path = tracks_path.with_name(tracks_path.stem + "Meta.csv")
            recording_meta_path = tracks_path.with_name(
                tracks_path.stem.split("_")[0] + "_recordingMeta.csv"
            )
            vehicle_tracks = pd.read_csv(tracks_path)
            tracks_meta = pd.read_csv(tracks_meta_path)
            recording_meta = pd.read_csv(recording_meta_path)

            vehicle_tracks.rename(
                columns={
                    "xCenter": "x",
                    "yCenter": "y",
                    "lonVelocity": "velocity",
                    "lonAcceleration": "acceleration",
                },
                inplace=True,
            )
            # Normalize orientation
            vehicle_tracks["heading"] = np.deg2rad(vehicle_tracks["heading"])
            vehicle_tracks["orientation"] = np.arctan2(
                np.sin(vehicle_tracks["heading"]), np.cos(vehicle_tracks["heading"])
            )

            # Rename tracks meta
            tracks_meta.rename(
                columns={
                    "class": "obstacle_type",
                },
                inplace=True,
            )
            tracks_meta["obstacle_type"].replace(self.obstacle_types, inplace=True)

            vehicle_tracks.set_index(["trackId", "frame"], inplace=True)
            tracks_meta.set_index(["trackId"], inplace=True)

            yield Window(
                vehicle_tracks[["x", "y", "velocity", "orientation", "acceleration"]],
                tracks_meta,
                get_dt(recording_meta),
            ), LevelXRecording(recording_meta.iloc[0].to_dict())


class LevelXConverterFactory(
    TabularConverterFactory[LevelXRecording], metaclass=ABCMeta
):
    input_dir: Path
    _config: Mapping[str, Any] = PrivateAttr()

    def __init__(self, **data: Any) -> None:
        super().__init__(**data)
        self._config = load_yaml(str(self.get_config_path()))

    @abstractmethod
    def get_config_path(self) -> Path:
        pass

    def build_recording_generator(self) -> LevelXRecordingGenerator:
        return LevelXRecordingGenerator(
            self.input_dir,
            self._config["locations"],
            self._config["class_to_obstacleType"],
        )

    def build_meta_scenario_creator(self) -> LevelXMetaScenarioGenerator:
        return LevelXMetaScenarioGenerator(
            self.get_config_path().parent / "maps", self._config["locations"]
        )
