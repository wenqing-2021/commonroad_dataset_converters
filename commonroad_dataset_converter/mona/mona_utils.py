import enum
import importlib.resources
import re
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
import pyproj
import scipy.spatial
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.scenario import Scenario, ScenarioID
from pyproj import Transformer

from commonroad_dataset_converter.helper import load_yaml, IJobGenerator, JobProcessor, T, TrackIdJobGenerator, \
    DisjunctiveJobGenerator


class MONALocation(enum.Enum):
    east = "east"
    west = "west"
    merge = "merge"


class MONADay(enum.Enum):
    d20210802 = "20210802"
    d20210803 = "20210803"
    d20210804 = "20210804"
    d20210805 = "20210805"
    d20210806 = "20210806"


def transform_coordinates(track_origin, df, map_scenario):
    # Transform coordinates
    trans = get_transform_to_scenario(map_scenario, track_origin)
    xy = np.array(list(trans.itransform(zip(df.x, df.y))))
    df.x = xy[:, 0]
    df.y = xy[:, 1]


def get_transform_to_scenario(map_scenario: Scenario, source_origin: np.ndarray, rotation=0) -> pyproj.Transformer:
    """
    Get a pyproj.Transformer instance to transform from trajectory coordinates to
    CommonRoad scenario coordinates

    :param map_scenario: CommonRoad scenario containing the map
    :param source_origin: Origin of the trajectories
    :return: pyproj.Transformer from trajectory coordinates to scenario coordinates
    """
    map_proj_string = map_scenario.location.geo_transformation.geo_reference
    match = re.findall(r"lat_0=(\d+\.\d+)\s.*lon_0=(\d+\.\d+)", map_proj_string)[0]
    map_proj_string = f"+proj=tmerc +lat_0={match[0]} +lon_0={match[1]} +datum=WGS84"
    rot = scipy.spatial.transform.Rotation.from_euler("z", rotation).as_matrix()

    trans = Transformer.from_pipeline(f"+proj=pipeline +step +proj=affine +xoff={source_origin[0]} +yoff="
                                      f"{source_origin[1]} +step +proj=utm inv +zone=32 +step {map_proj_string} "
                                      f"+step +proj=affine +s11={rot[0, 0]} +s12={rot[0, 1]} +s21="
                                      f"{rot[1, 0]} +s22={rot[1, 1]}")
    return trans


def _get_mona_map(config, location):
    with importlib.resources.path("commonroad_dataset_converter.mona.maps", config["maps"][location]) as map_path:
        scenario, _ = CommonRoadFileReader(str(map_path)).open()
    return scenario


def _read_trajectories(trajectory_file: Path) -> pd.DataFrame:
    assert trajectory_file.exists()
    if trajectory_file.suffix == ".csv":
        df = pd.read_csv(trajectory_file)
    else:
        df = pd.read_parquet(trajectory_file)
    return df


def _get_mona_config():
    with importlib.resources.path("commonroad_dataset_converter.mona", "config.yaml") as config_path:
        config = load_yaml(str(config_path))
    return config


@dataclass
class MONAJob:
    df: pd.DataFrame
    map_suffix: str
    track_id: Optional[int] = None


file_name_pattern = re.compile(
        r"(?P<location>east|west|merge)_(?P<day>2021080[2-6])_(?P<segment>[0-9]{3})_trajectories")


class MONATrackIdJobGenerator(IJobGenerator):

    def __init__(self, trajectory_file: Path, location: MONALocation, map_scenario: Scenario, config,
                 num_scenarios: int, track_id: int):
        self.trajectory_file = trajectory_file
        self.map_scenario = map_scenario
        self.num_scenarios = num_scenarios
        self.track_id = track_id
        self.origin = config["origin"]
        match = file_name_pattern.match(trajectory_file.stem)
        if match is not None:
            if match["location"] != location.value:
                warnings.warn(f"Using different map than trajectory location! {match['location']} != {location.value}")
            map_suffix = f'{match["day"]}{match["segment"]}'
        else:
            map_suffix = ""
        self.map_suffix = map_suffix

    def __iter__(self) -> T:
        traj_df = _read_trajectories(self.trajectory_file)
        transform_coordinates(self.origin, traj_df, self.map_scenario)
        generator = TrackIdJobGenerator(traj_df, self.num_scenarios, self.track_id)
        for tid, df in generator:
            yield MONAJob(df, self.map_suffix, tid)


class MONADisjunctiveJobGenerator(IJobGenerator):
    def __init__(self, trajectory_file: Path, location: MONALocation, map_scenario: Scenario, config,
                 num_scenarios: int, max_duration: int):
        self.trajectory_file = trajectory_file
        self.map_scenario = map_scenario
        self.num_scenarios = num_scenarios
        self.origin = config["origin"]
        match = file_name_pattern.match(trajectory_file.stem)
        if match is not None:
            if match["location"] != location.value:
                warnings.warn(f"Using different map than trajectory location! {match['location']} != {location.value}")
            map_suffix = f'{match["day"]}{match["segment"]}'
        else:
            map_suffix = ""
        self.map_suffix = map_suffix
        self.max_duration = max_duration

    def __iter__(self) -> MONAJob:
        traj_df = _read_trajectories(self.trajectory_file)
        transform_coordinates(self.origin, traj_df, self.map_scenario)
        generator = DisjunctiveJobGenerator(traj_df, self.num_scenarios, self.max_duration)
        for df in generator:
            yield MONAJob(df, self.map_suffix)

    def __len__(self):
        return self.num_scenarios


@dataclass
class MONAProcessor(JobProcessor):

    def _get_scenario_id(self, job: MONAJob) -> ScenarioID:
        scenario_id = super()._get_scenario_id(job)
        if self.disjunctive:
            scenario_id.map_name = scenario_id.map_name[:-1] + job.map_suffix
        else:
            scenario_id.map_name += job.map_suffix
        return scenario_id

    def __call__(self, job: MONAJob) -> None:
        time_steps = (job.df.frame_id.drop_duplicates().sort_values().to_frame().reset_index(drop=True).reset_index(
                drop=False).rename(columns={"index": "time_step"}))
        job.df = job.df.merge(time_steps, on="frame_id")
        super().__call__(job)
