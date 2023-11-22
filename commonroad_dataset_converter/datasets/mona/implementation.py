import copy
import importlib.resources
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import ClassVar, Dict, Iterable, Tuple

import numpy as np
import pandas as pd
import pyproj
import scipy.spatial
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.scenario import Scenario
from pyproj import Transformer

from commonroad_dataset_converter.conversion.tabular.interface import (
    IMetaScenarioCreator,
    IRecordingGenerator,
    Window,
)
from commonroad_dataset_converter.datasets.INTERACTION.implementation import (
    InteractionRecording,
)

_folder_pattern = re.compile(
    r"(?P<location>east|west|merge)[_-](?P<day>00[0-4])[_-](?P<segment>[0-9]{3})"
)


_proj_string = {
    "east": "+proj=tmerc +lat_0=48.17722803371593 +lon_0=11.59292664305758 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m "
    "+geoidgrids=egm96_15.gtx +vunits=m +no_defs",
    "west": "+proj=tmerc +lat_0=48.17722803371593 +lon_0=11.59292664305758 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m "
    "+geoidgrids=egm96_15.gtx +vunits=m +no_defs",
    "merge": "+proj=tmerc +lat_0=48.17722803371593 +lon_0=11.59292664305758 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m "
    "+geoidgrids=egm96_15.gtx +vunits=m +no_defs",
}
_track_origin = np.array([692000.0, 5340000.0])


def transform_coordinates(df: pd.DataFrame, location: str) -> None:
    # Transform coordinates
    trans = get_transform_to_scenario(_proj_string[location], _track_origin)
    xy = np.array(list(trans.itransform(zip(df.x, df.y))))
    df.x = xy[:, 0]
    df.y = xy[:, 1]


def get_transform_to_scenario(
    map_proj_string: str, source_origin: np.ndarray, rotation: float = 0
) -> pyproj.Transformer:
    """
    Get a pyproj.Transformer instance to transform from trajectory coordinates to
    CommonRoad scenario coordinates

    :param map_proj_string: Map projection string
    :param source_origin: Origin of the trajectories
    :return: pyproj.Transformer from trajectory coordinates to scenario coordinates
    """
    match = re.findall(r"lat_0=(\d+\.\d+)\s.*lon_0=(\d+\.\d+)", map_proj_string)[0]
    map_proj_string = f"+proj=tmerc +lat_0={match[0]} +lon_0={match[1]} +datum=WGS84"
    rot = scipy.spatial.transform.Rotation.from_euler("z", rotation).as_matrix()

    trans = Transformer.from_pipeline(
        f"+proj=pipeline +step +proj=affine +xoff={source_origin[0]} +yoff="
        f"{source_origin[1]} +step +proj=utm inv +zone=32 +step {map_proj_string} "
        f"+step +proj=affine +s11={rot[0, 0]} +s12={rot[0, 1]} +s21="
        f"{rot[1, 0]} +s22={rot[1, 1]}"
    )
    return trans


def _read_trajectories(trajectory_file: Path) -> pd.DataFrame:
    assert trajectory_file.exists()
    if trajectory_file.suffix == ".csv":
        df = pd.read_csv(trajectory_file)
    else:
        df = pd.read_parquet(trajectory_file)
    return df


@dataclass
class MonaMetaScenarioGenerator(IMetaScenarioCreator):
    map_names: ClassVar[Dict[str, str]] = {
        "east": "DEU_MONAEast-2.xml",
        "west": "DEU_MONAWest-2.xml",
        "merge": "DEU_MONAMerge-2.xml",
    }
    _name_to_map: Dict[str, Scenario] = field(default_factory=dict, init=False)

    def __post_init__(self) -> None:
        for location in ["east", "west", "merge"]:
            with importlib.resources.path(
                "commonroad_dataset_converter.datasets.mona.maps",
                self.map_names[location],
            ) as map_path:
                lanelet_network = CommonRoadFileReader(str(map_path)).open()[0]
            self._name_to_map[location] = lanelet_network

    def __call__(
        self, recording: Window, window_meta: InteractionRecording
    ) -> Scenario:
        scenario = copy.deepcopy(self._name_to_map[window_meta.location])
        scenario.scenario_id.configuration_id = window_meta.recording_id
        return scenario


@dataclass
class MonaRecordingGenerator(IRecordingGenerator):
    data_path: Path

    def __iter__(self) -> Iterable[Tuple[Window, InteractionRecording]]:
        if self.data_path.is_dir():
            files = sorted(list(self.data_path.rglob("trajectories.*")))
        else:
            files = [self.data_path]

        for trajectory_file in files:
            match = _folder_pattern.match(trajectory_file.parent.name)

            assert match is not None, f"Invalid file name {trajectory_file.stem}!"

            traj_df = _read_trajectories(trajectory_file)
            transform_coordinates(traj_df, match["location"])
            traj_df.rename(
                columns={
                    "agent_type": "obstacle_type",
                    "v": "velocity",
                    "psi_rad": "orientation",
                },
                inplace=True,
            )
            traj_df.set_index(["track_id", "frame_id"], inplace=True)
            traj_meta = traj_df.groupby(level=0)[
                ["obstacle_type", "length", "width"]
            ].first()

            yield Window(
                traj_df[["x", "y", "velocity", "orientation", "yaw_rate"]],
                traj_meta,
                0.04,
            ), InteractionRecording(
                location=match["location"],
                recording_id=int(match["day"] + match["segment"]),
            )
