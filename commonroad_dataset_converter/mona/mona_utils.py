import enum
import importlib.resources
import math
import os
import re
from dataclasses import dataclass
from multiprocessing import Pool
from pathlib import Path
from typing import Optional, Dict, Any

import numpy as np
import pandas as pd
import pyproj
import requests
import scipy.spatial
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.scenario.scenario import Scenario, ScenarioID, Tag
from commonroad.scenario.trajectory import State, Trajectory
from pyproj import Transformer
from tqdm import tqdm

from commonroad_dataset_converter.helper import load_yaml
from commonroad_dataset_converter.planning_problem_utils import (obstacle_to_planning_problem, )


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


def create_obstacle(track_df: pd.DataFrame, dynamic_obstacle_id):
    length = track_df.length.values[0]
    width = track_df.width.values[0]

    dynamic_obstacle_type = ObstacleType(track_df.agent_type.iloc[0])
    dynamic_obstacle_shape = Rectangle(width=width, length=length)

    time_steps = np.array(track_df.time_step)
    xs = np.array(track_df.x)
    ys = np.array(track_df.y)
    velocities = np.array(track_df.v)
    orientations = np.array(track_df.psi_rad)

    state_list = []
    for t, x, y, v, theta in zip(time_steps, xs, ys, velocities, orientations):
        state_list.append(State(position=np.array([x, y]), velocity=v, orientation=theta, time_step=int(t), ))

    dynamic_obstacle_initial_state = state_list[0]

    dynamic_obstacle_trajectory = Trajectory(state_list[1].time_step, state_list[1:])
    dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, dynamic_obstacle_shape)

    return DynamicObstacle(dynamic_obstacle_id, dynamic_obstacle_type, dynamic_obstacle_shape,
                           dynamic_obstacle_initial_state, dynamic_obstacle_prediction, )


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


def _get_map(config, location):
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


class MONATrackIdJobGenerator:
    def __init__(self, traj_df, num_scenarios, track_id):
        if track_id is not None:
            track_ids = [track_id]
        elif num_scenarios is not None:
            track_ids = np.random.choice(traj_df.track_id.unique(), num_scenarios, replace=False)
        else:
            track_ids = traj_df.track_id.unique()

        self.track_ids = track_ids
        self.traj_df = traj_df

    def __iter__(self):
        for tid in self.track_ids:
            track = self.traj_df[self.traj_df.track_id == tid]
            start_frame = track.frame_id.min()
            end_frame = track.frame_id.max() + 1
            yield MONAJob(track_id=tid, df=self.traj_df[
                (self.traj_df.frame_id >= start_frame) & (self.traj_df.frame_id < end_frame)], )

    def __len__(self):
        return len(self.track_ids)


class MONADisjunctiveJobGenerator:
    def __init__(self, traj_df, num_scenarios, max_duration):
        num_time_steps = traj_df.frame_id.max() - traj_df.frame_id.min() + 1
        if num_scenarios is None:
            # Either num_scenarios or max_duration must be specified!
            max_duration = max_duration or 100
            num_scenarios = math.ceil(num_time_steps / max_duration)
        elif max_duration is None:
            max_duration = math.floor(num_time_steps / num_scenarios)
        self.max_duration = max_duration
        self.num_scenarios = num_scenarios
        self.traj_df = traj_df

    def __iter__(self):
        for i in range(self.num_scenarios):
            yield MONAJob(self.traj_df[(self.traj_df.frame_id >= i * self.max_duration) & (
                    self.traj_df.frame_id < (i + 1) * self.max_duration)])

    def __len__(self):
        return self.num_scenarios


@dataclass
class MONAJob:
    df: pd.DataFrame
    track_id: Optional[int] = None


@dataclass
class MONAProcessor:
    map_scenario: Scenario
    config: Dict[str, Any]
    output_dir: Path
    keep_ego: bool
    map_suffix: str

    def __call__(self, job: MONAJob) -> None:
        ego_id = job.track_id
        traj_df = job.df
        time_steps = (traj_df.frame_id.drop_duplicates().sort_values().to_frame().reset_index(drop=True).reset_index(
                drop=False).rename(columns={"index": "time_step"}))
        traj_df = traj_df.merge(time_steps, on="frame_id")
        dt = np.around(np.mean(np.diff(np.sort(traj_df.unix_timestamp.unique()))), 2)
        map_name = self.map_scenario.scenario_id.map_name + self.map_suffix
        if ego_id is None:
            traj_lens = traj_df.groupby("track_id").time_step.count()
            ego_id = traj_lens.index[traj_lens.argmax()]
            # Add a suffix D indicating disjunctive scenarios
            map_name += "D"
        start_frame = traj_df.frame_id.min()
        scenario = Scenario(dt=float(dt), scenario_id=ScenarioID(country_id=self.map_scenario.scenario_id.country_id,
                                                                 map_name=map_name,
                                                                 map_id=self.map_scenario.scenario_id.map_id,
                                                                 obstacle_behavior="T",
                                                                 configuration_id=start_frame + 1,
                                                                 prediction_id=ego_id, ),
                            location=self.map_scenario.location, )
        scenario.add_objects(self.map_scenario.lanelet_network)

        # Create obstacles
        for track_id in traj_df.track_id.unique():
            track_df = traj_df[traj_df.track_id == track_id]
            if len(track_df) < 2:
                continue
            obs = create_obstacle(track_df, int(track_id) + 1000)
            scenario.add_objects(obs)

        ego_obstacle = scenario.obstacle_by_id(int(ego_id) + 1000)
        if not self.keep_ego:
            scenario.remove_obstacle(ego_obstacle)

        planning_problem = obstacle_to_planning_problem(ego_obstacle, planning_problem_id=ego_id,
                                                        lanelet_network=scenario.lanelet_network, )

        if len(scenario.dynamic_obstacles) == 0:
            return

        # Write scenario file
        cw = CommonRoadFileWriter(scenario, PlanningProblemSet([planning_problem]), author=self.config["author"],
                                  affiliation=self.config["affiliation"], source=self.config["source"],
                                  tags={Tag.URBAN}, )
        cw.write_to_file(str(self.output_dir / f"{scenario.scenario_id}.xml"), OverwriteExistingFile.ALWAYS, )


def run_processor(generator, processor, num_processes):
    try:
        num_scenarios = len(generator)
        num_processes = min(num_processes, num_scenarios)
    except TypeError:
        num_scenarios = None

    if num_processes > 1:
        with Pool(num_processes) as pool:
            iterator = pool.imap_unordered(processor, generator)
            for _ in tqdm(iterator, total=num_scenarios, desc="Creating scenarios", unit="scenario", ):
                pass
    else:
        for params in tqdm(generator, total=num_scenarios, desc="Creating scenarios", unit="scenario"):
            processor(params)
