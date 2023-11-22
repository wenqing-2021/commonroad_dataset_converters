import copy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Tuple

import numpy as np
import pandas as pd
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.scenario import Scenario

from commonroad_dataset_converter.conversion.tabular.interface import (
    IMetaScenarioCreator,
    IRecordingGenerator,
    Window,
)


@dataclass
class InteractionRecording:
    location: str
    recording_id: int


@dataclass
class InteractionMetaScenarioGenerator(IMetaScenarioCreator):
    map_names: Dict[str, str]
    _name_to_map: Dict[str, Scenario] = field(default_factory=dict, init=False)

    def __post_init__(self) -> None:
        path_prefix = Path(__file__).parent / "maps"
        for location, map_name in self.map_names.items():
            lanelet_network = CommonRoadFileReader(
                str(path_prefix / (map_name + ".xml"))
            ).open()[0]
            self._name_to_map[location] = lanelet_network

    def __call__(
        self, recording: Window, window_meta: InteractionRecording
    ) -> Scenario:
        meta_scenario = copy.deepcopy(self._name_to_map[window_meta.location])
        meta_scenario.scenario_id.configuration_id = window_meta.recording_id
        return meta_scenario


@dataclass
class InteractionRecordingGenerator(IRecordingGenerator):
    data_path: Path
    locations: Dict[str, str]
    offsets: Dict[str, Tuple[float, float]]

    def __iter__(self) -> Iterable[Tuple[Window, InteractionRecording]]:
        for location, path in self.locations.items():
            location_path = self.data_path / path

            if not location_path.exists():
                continue

            for trajectory_file in location_path.glob("*.csv"):
                recording_id = int(trajectory_file.stem.split("_")[-1])
                traj_df = pd.read_csv(trajectory_file)
                # Translate
                traj_df["x"] -= self.offsets[location][0]
                traj_df["y"] -= self.offsets[location][1]

                traj_df["velocity"] = np.cos(traj_df.psi_rad) * traj_df.vx + np.sin(
                    traj_df.psi_rad
                ) * (-traj_df.vy)

                traj_df.rename(
                    columns={
                        "agent_type": "obstacle_type",
                        "psi_rad": "orientation",
                    },
                    inplace=True,
                )
                traj_df.set_index(["track_id", "frame_id"], inplace=True)
                traj_meta = traj_df.groupby(level=0)[
                    ["obstacle_type", "length", "width"]
                ].first()

                yield Window(
                    traj_df[["x", "y", "velocity", "orientation"]], traj_meta, 0.04
                ), InteractionRecording(location, recording_id)
