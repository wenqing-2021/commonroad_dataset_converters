import math
from abc import ABCMeta, abstractmethod
from dataclasses import dataclass
from multiprocessing import Pool
from pathlib import Path
from typing import Dict, Union, TypeVar, Generic, Iterable, Sequence, Callable, Any, Tuple

import numpy as np
import pandas as pd
from _ruamel_yaml import YAMLError
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.common.util import make_valid_orientation, make_valid_orientation_interval
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.scenario.scenario import ScenarioID, Scenario, Tag
from commonroad.scenario.trajectory import State, Trajectory
from ruamel.yaml import YAML
from tqdm import tqdm

from commonroad_dataset_converter.planning_problem_utils import obstacle_to_planning_problem, NoCarException


def load_yaml(file_name: str) -> Union[Dict, None]:
    """
    Loads configuration setup from a yaml file

    :param file_name: name of the yaml file
    """
    with open(file_name, 'r') as stream:
        try:
            config = YAML().load(stream)
            return config
        except YAMLError as exc:
            print(exc)
            return None


def make_valid_orientation_pruned(orientation: float):
    """
    Make orientation valid and prune to correct representation for XML with 6 significant digits
    """
    orientation = make_valid_orientation(orientation)
    return max(min(orientation, 6.283185), -6.283185)


def make_valid_orientation_interval_pruned(o1: float, o2: float):
    """
    Make orientation valid and prune to correct representation for XML with 6 significant digits
    """
    o1, o2 = make_valid_orientation_interval(o1, o2)
    return make_valid_orientation_pruned(o1), make_valid_orientation_pruned(o2)


T = TypeVar("T")


class IJobGenerator(Generic[T], metaclass=ABCMeta):
    @abstractmethod
    def __iter__(self) -> T:
        pass


class IJobProcessor(Generic[T], metaclass=ABCMeta):
    @abstractmethod
    def __call__(self, job: T) -> None:
        pass


@dataclass
class JobProcessor(IJobProcessor[T]):
    map_scenario: Scenario
    config: Dict[str, Any]
    output_dir: Path
    keep_ego: bool
    disjunctive: bool
    dt: float

    def _get_scenario_id(self, job: T) -> ScenarioID:
        start_frame = job.df.frame_id.min()
        map_name = self.map_scenario.scenario_id.map_name
        # Add a suffix D indicating disjunctive scenarios
        if self.disjunctive:
            map_name += "D"
        return ScenarioID(country_id=self.map_scenario.scenario_id.country_id, map_name=map_name,
                          map_id=self.map_scenario.scenario_id.map_id, obstacle_behavior="T",
                          configuration_id=start_frame + 1, prediction_id=job.track_id, )

    def _create_scenario(self, job):
        traj_df = job.df.copy()
        traj_df.loc[:, "time_step"] -= traj_df.time_step.min()
        scenario = Scenario(dt=self.dt, scenario_id=self._get_scenario_id(job), location=self.map_scenario.location)
        scenario.add_objects(self.map_scenario.lanelet_network)

        if job.track_id is None:
            candidates = traj_df[traj_df.agent_type.isin(["car"])]
            if len(candidates.index) == 0:
                return scenario, None
            traj_lens = candidates.groupby("track_id").time_step.count()
            job.track_id = traj_lens.index[traj_lens.argmax()]

        scenario.scenario_id.prediction_id = job.track_id

        # Create obstacles
        for track_id in traj_df.track_id.unique():
            track_df = traj_df[traj_df.track_id == track_id]
            if len(track_df) < 2:
                continue
            obs = create_obstacle(track_df, int(track_id) + 1000)
            scenario.add_objects(obs)
        ego_obstacle = scenario.obstacle_by_id(int(job.track_id) + 1000)
        if not self.keep_ego:
            scenario.remove_obstacle(ego_obstacle)

        try:
            final_time_step = max((s.time_step for s in ego_obstacle.prediction.trajectory.state_list if
                                   len(scenario.lanelet_network.find_lanelet_by_position([s.position])[0]) > 0),
                                  default=-1)
            if final_time_step - ego_obstacle.initial_state.time_step <= 25:
                raise NoCarException
            final_time_step += 25

            planning_problem = obstacle_to_planning_problem(ego_obstacle, planning_problem_id=job.track_id,
                                                            lanelet_network=scenario.lanelet_network,
                                                            final_time_step=final_time_step)
        except NoCarException:
            planning_problem = None
        return scenario, planning_problem

    def _save_scenario(self, scenario, planning_problem):
        planning_problem_list = [planning_problem] if planning_problem is not None else None
        cw = CommonRoadFileWriter(scenario, PlanningProblemSet(planning_problem_list), author=self.config["author"],
                                  affiliation=self.config["affiliation"], source=self.config["source"],
                                  tags={Tag.URBAN}, )
        cw.write_to_file(str(self.output_dir / f"{scenario.scenario_id}.xml"), OverwriteExistingFile.ALWAYS, )

    def __call__(self, job) -> None:
        scenario, planning_problem = self._create_scenario(job)

        if len(scenario.dynamic_obstacles) == 0:
            return

        # Write scenario file
        self._save_scenario(scenario, planning_problem)


def run_processor(generator: Union[Iterable[T], Sequence[T]], processor: Callable[[T], None], num_processes: int):
    try:
        num_scenarios = len(generator)
        num_processes = min(num_processes, num_scenarios)
    except TypeError:
        num_scenarios = None

    if num_processes > 1:
        with Pool(num_processes) as pool:
            iterator = pool.imap_unordered(processor, generator)
            for _ in tqdm(iterator, total=num_scenarios, desc="Creating scenarios", unit="scenarios", ):
                pass
    else:
        for params in tqdm(generator, total=num_scenarios, desc="Creating scenarios", unit="scenarios"):
            processor(params)


def create_obstacle(track_df: pd.DataFrame, dynamic_obstacle_id):
    length = track_df.length.values[0]
    width = track_df.width.values[0]

    dynamic_obstacle_type = ObstacleType(track_df.agent_type.iloc[0])
    if dynamic_obstacle_type == ObstacleType.PEDESTRIAN:
        dynamic_obstacle_shape = Circle(0.5)
    else:
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


class TrackIdJobGenerator(IJobGenerator):
    def __init__(self, traj_df, num_scenarios, track_id):
        valid_ids = traj_df[traj_df.agent_type.isin(["car"])].track_id.unique()
        if track_id is not None:
            assert track_id in valid_ids, f"Track ID {track_id} is invalid!"
            track_ids = [track_id]
        elif num_scenarios is not None:
            track_ids = np.random.choice(valid_ids, num_scenarios, replace=False)
        else:
            track_ids = valid_ids

        self.track_ids = track_ids
        self.traj_df = traj_df

    def __iter__(self) -> Tuple[int, pd.DataFrame]:
        for tid in self.track_ids:
            track = self.traj_df[self.traj_df.track_id == tid]
            start_frame = track.frame_id.min()
            end_frame = track.frame_id.max() + 1
            yield tid, self.traj_df[(self.traj_df.frame_id >= start_frame) & (self.traj_df.frame_id < end_frame)]

    def __len__(self):
        return len(self.track_ids)


class DisjunctiveJobGenerator(IJobGenerator):
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

    def __iter__(self) -> pd.DataFrame:
        for i in range(self.num_scenarios):
            yield self.traj_df[(self.traj_df.frame_id >= i * self.max_duration) & (
                        self.traj_df.frame_id < (i + 1) * self.max_duration)]

    def __len__(self):
        return self.num_scenarios
