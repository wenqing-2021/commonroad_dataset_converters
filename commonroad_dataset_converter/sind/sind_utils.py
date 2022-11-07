import importlib.resources
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple, Optional

import numpy as np
import pandas as pd
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.scenario import ScenarioID
from commonroad.scenario.traffic_sign import (TrafficLight, TrafficLightCycleElement, TrafficLightState, )

from commonroad_dataset_converter.helper import load_yaml, JobProcessor, IJobGenerator, T, TrackIdJobGenerator, \
    DisjunctiveJobGenerator

state_map = {
    0: TrafficLightState.RED,
    1: TrafficLightState.GREEN,
    3: TrafficLightState.YELLOW,
}


@dataclass
class SINDJob:
    df: pd.DataFrame
    tl_df: pd.DataFrame
    recording_name: str
    track_id: Optional[int] = None


@dataclass
class SinDProcessor(JobProcessor):

    def _get_scenario_id(self, job: SINDJob) -> ScenarioID:
        scenario_id = super()._get_scenario_id(job)
        scenario_id.map_id = int(job.recording_name)
        return scenario_id

    def __call__(self, job: SINDJob) -> None:
        traj_df = job.df
        tl_df = job.tl_df
        # Traffic lights
        tl_states = _prepare_traffic_light_df(tl_df, traj_df)

        traj_df = traj_df.copy()
        start_time_step = traj_df.time_step.min()
        traj_df.time_step = traj_df.time_step - start_time_step
        self.dt = round(
            float((traj_df.timestamp_ms.max() - traj_df.timestamp_ms.min()) / (traj_df.time_step.max())) / 1000.0, 2)

        scenario, planning_problem = super()._create_scenario(job)

        _add_traffic_lights(scenario, tl_states, self.config)

        super()._save_scenario(scenario, planning_problem)


@dataclass
class SindDisjunctiveJobGenerator(IJobGenerator):
    recording_dir: Path
    num_scenarios: Optional[int]
    max_duration: Optional[int]
    recording_name: str

    def __iter__(self) -> T:
        traj_df, tl_df = _read_trajectories(self.recording_dir)
        generator = DisjunctiveJobGenerator(traj_df, self.num_scenarios, self.max_duration)
        for df in generator:
            yield SINDJob(df, tl_df, self.recording_name)


@dataclass
class SindTrackIdJobGenerator(IJobGenerator):
    recording_dir: Path
    num_scenarios: Optional[int]
    max_duration: Optional[int]
    recording_name: str

    def __iter__(self) -> T:
        traj_df, tl_df = _read_trajectories(self.recording_dir)
        generator = TrackIdJobGenerator(traj_df, self.num_scenarios, self.max_duration)
        for tid, df in generator:
            yield SINDJob(df, tl_df, self.recording_name, tid)


def _add_traffic_lights(scenario, tl_states, config):
    for t in scenario.lanelet_network.traffic_lights:
        try:
            # TODO: This seems to be a bug in commonroad-io!
            scenario.remove_traffic_light(t)
        except KeyError:
            continue
    for tl_index, tl_position in config["traffic_light_positions"].items():
        cycle_elems = [
            TrafficLightCycleElement(state_map[state_index], duration)
            for state_index, duration in zip(
                tl_states[f"Traffic light {tl_index}"].values, tl_states.duration.values
            )
        ]
        tl_id = tl_index + 100
        scenario.add_objects(
            TrafficLight(tl_id, cycle_elems, position=np.array(tl_position))
        )
        for incoming_id in config["traffic_light_incomings"][tl_index]:
            lanelet = scenario.lanelet_network.find_lanelet_by_id(incoming_id)
            lanelet.add_traffic_light_to_lanelet(tl_id)
            assert lanelet.stop_line is not None, f"Lanelet {incoming_id} does not have a stop line!"
            lanelet.stop_line.traffic_light_ref = {tl_id}


def _prepare_traffic_light_df(tl_series, traj_df):
    tl_init_state = tl_series[tl_series.frame_id <= traj_df.frame_id.min()].iloc[-1:]
    tl_init_state.frame_id = traj_df.frame_id.min()
    tl_states = tl_series[
        (tl_series.frame_id >= traj_df.frame_id.min())
        & (tl_series.frame_id <= traj_df.frame_id.max())
    ]
    tl_states = pd.concat((tl_init_state, tl_states))
    tl_states.drop_duplicates(inplace=True)
    tl_states["duration"] = np.diff(
        tl_states.frame_id.values, append=traj_df.frame_id.max()
    )
    return tl_states


def _get_map():
    with importlib.resources.path(
        "commonroad_dataset_converter.sind", "CHN_SinD-1.xml"
    ) as map_path:
        map_scenario = CommonRoadFileReader(str(map_path)).open()[0]
    return map_scenario


def _read_dataframe(data_frame_file: Path) -> pd.DataFrame:
    assert data_frame_file.exists(), f"{data_frame_file.absolute()} does not exist!"
    if data_frame_file.suffix == ".csv":
        df = pd.read_csv(data_frame_file)
    else:
        df = pd.read_parquet(data_frame_file)
    return df


def _read_trajectories(dataset_dir: Path) -> Tuple[pd.DataFrame, pd.DataFrame]:
    traj_df = _read_dataframe(dataset_dir / "Veh_smoothed_tracks.csv")
    traj_df.loc[traj_df.agent_type == "tricycle", "agent_type"] = ObstacleType.MOTORCYCLE.value

    ped_df = _read_dataframe(dataset_dir / "Ped_smoothed_tracks.csv")
    new_track_id = (
        ped_df[["track_id"]].drop_duplicates().reset_index(drop=True).reset_index()
    )
    ped_df = ped_df.merge(new_track_id, on="track_id")
    ped_df.drop(columns=["track_id"], inplace=True)
    ped_df.rename(columns={"index": "track_id"}, inplace=True)

    ped_df.track_id += 1 + traj_df.track_id.max()
    ped_df["v_lon"] = np.linalg.norm(ped_df[["vx", "vy"]].values, axis=1)
    ped_df["yaw_rad"] = np.arctan2(ped_df.vy, ped_df.vx)

    traj_df = pd.concat((traj_df, ped_df), ignore_index=True)
    traj_df.rename(columns={"yaw_rad": "psi_rad", "v_lon": "v"}, inplace=True)
    traj_df["time_step"] = traj_df.frame_id

    dataset_ids = dataset_dir.name.split("_")
    dataset_ids[1] = f"{int(dataset_ids[1]):02d}"
    tl_df = pd.read_csv(dataset_dir / f"TrafficLight_{'_'.join(dataset_ids)}.csv")
    tl_df.dropna(inplace=True)
    tl_df["frame_id"] = np.around(tl_df.RawFrameID / 3.0).astype(int)
    traj_df = traj_df[traj_df.agent_type.isin([v.value for v in ObstacleType])]
    return traj_df, tl_df


def _get_sind_config():
    with importlib.resources.path(
        "commonroad_dataset_converter.sind", "config.yaml"
    ) as config_path:
        config = load_yaml(str(config_path))
    return config
