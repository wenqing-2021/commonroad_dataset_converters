import copy
import importlib.resources
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Sequence, Tuple

import numpy as np
import pandas as pd
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_light import (
    TrafficLight,
    TrafficLightCycle,
    TrafficLightCycleElement,
    TrafficLightState,
)

from commonroad_dataset_converter.conversion.tabular.interface import (
    IMetaScenarioCreator,
    IRecordingGenerator,
    Window,
)
from commonroad_dataset_converter.conversion.tabular.prototype_scenario import (
    ScenarioPrototypeCreator,
)

state_map = {
    0: TrafficLightState.RED,
    1: TrafficLightState.GREEN,
    3: TrafficLightState.YELLOW,
}


@dataclass
class SindWindowMeta:
    traffic_light_states: pd.DataFrame
    recording_id: str


@dataclass
class SindRecordingGenerator(IRecordingGenerator):
    data_path: Path
    downsample: int

    def _read_trajectories(
        self,
        dataset_dir: Path,
    ) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
        # Read vehicle trajectories
        vehicle_df = pd.read_csv(dataset_dir / "Veh_smoothed_tracks.csv")
        # We don't have a type for tricycle. Replace with motorcycle.
        vehicle_df["agent_type"].replace(
            {"tricycle": ObstacleType.MOTORCYCLE.value}, inplace=True
        )
        vehicle_df.set_index("track_id", inplace=True)

        # Read pedestrian trajectories
        ped_df = pd.read_csv(dataset_dir / "Ped_smoothed_tracks.csv")
        # Assign a numeric id to pedestrians, starting from the highest
        # vehicle id.
        ped_df.set_index(
            ped_df.set_index("track_id").index.factorize()[0]
            + 1
            + vehicle_df.index.max(),
            inplace=True,
        )
        ped_df.drop(columns=["track_id"], inplace=True)

        # Compute longitudinal speed for pedestrians
        ped_df["v_lon"] = np.linalg.norm(ped_df[["vx", "vy"]].values, axis=1)
        # Compute orientation for pedestrians
        ped_df["yaw_rad"] = np.arctan2(ped_df.vy, ped_df.vx)

        # Combine trajectories of pedestrians and vehicles
        traj_df = pd.concat((vehicle_df, ped_df))
        traj_df.rename(
            columns={
                "yaw_rad": "orientation",
                "v_lon": "velocity",
                "agent_type": "obstacle_type",
            },
            inplace=True,
        )
        traj_df.set_index("frame_id", append=True, inplace=True)
        # Filter obstacle types we can represent
        traj_df = traj_df[traj_df.obstacle_type.isin([v.value for v in ObstacleType])]
        # Split of obstacle meta data
        traj_meta = traj_df.groupby(level=0)[
            ["obstacle_type", "length", "width"]
        ].first()
        traj_df = traj_df[["x", "y", "velocity", "orientation"]]

        # Load traffic light states
        dataset_ids = dataset_dir.name.split("_")
        dataset_ids[1] = f"{int(dataset_ids[1]):02d}"
        tl_df = pd.read_csv(dataset_dir / f"TrafficLight_{'_'.join(dataset_ids)}.csv")
        # Some files are not ordered
        tl_df.sort_values("RawFrameID", inplace=True)
        tl_df.reset_index(drop=True, inplace=True)
        tl_df.dropna(inplace=True)
        tl_df["time_step"] = np.around(tl_df.RawFrameID / 3.0 / self.downsample).astype(
            int
        )

        return traj_df, traj_meta, tl_df

    def __iter__(self) -> Iterable[Tuple[Window, SindWindowMeta]]:
        assert (
            self.data_path.exists()
        ), f"{self.data_path.resolve().absolute()} does not exist!"
        assert (
            self.data_path.is_dir()
        ), f"{self.data_path.resolve().absolute()} is not a directory!"

        if all([p.is_file() for p in self.data_path.iterdir()]):
            recordings = [self.data_path]
        else:
            recordings = [p for p in self.data_path.iterdir() if p.is_dir()]

        for recording in recordings:
            traj_df, dynamic_meta, tl_df = self._read_trajectories(recording)
            yield Window(traj_df, dynamic_meta, 0.1), SindWindowMeta(
                tl_df, recording.stem
            )


@dataclass
class SindScenarioPrototypeGenerator(ScenarioPrototypeCreator):
    traffic_light_incomings: Dict[int, Sequence[int]]
    traffic_light_positions: Dict[int, Tuple[float, float]]

    def _add_traffic_lights(self, scenario: Scenario, tl_states: pd.DataFrame) -> None:
        for t in scenario.lanelet_network.traffic_lights:
            try:
                # TODO: This seems to be a bug in commonroad-io!
                scenario.remove_traffic_light(t)
            except KeyError:
                continue
        for tl_index, tl_position in self.traffic_light_positions.items():
            assert np.all(tl_states.duration.values >= 0)
            cycle_elems = [
                TrafficLightCycleElement(state_map[state_index], duration)
                for state_index, duration in zip(
                    tl_states[f"Traffic light {tl_index}"].values,
                    tl_states.duration.values,
                )
            ]
            tl_id = tl_index + 100
            cycle = TrafficLightCycle(cycle_elems)
            scenario.add_objects(
                TrafficLight(
                    traffic_light_id=tl_id,
                    traffic_light_cycle=cycle,
                    position=np.array(tl_position),
                )
            )
            for incoming_id in self.traffic_light_incomings[tl_index]:
                lanelet = scenario.lanelet_network.find_lanelet_by_id(incoming_id)
                lanelet.add_traffic_light_to_lanelet(tl_id)
                assert (
                    lanelet.stop_line is not None
                ), f"Lanelet {incoming_id} does not have a stop line!"
                lanelet.stop_line.traffic_light_ref = {tl_id}

    @staticmethod
    def _prepare_traffic_light_df(
        tl_series: pd.DataFrame, time_steps: np.ndarray
    ) -> pd.DataFrame:
        tl_init_state = tl_series[tl_series.time_step <= time_steps.min()].iloc[-1:]
        tl_init_state.time_step = time_steps.min()
        tl_states = tl_series[
            (tl_series.time_step >= time_steps.min())
            & (tl_series.time_step <= time_steps.max())
        ]
        tl_states = pd.concat((tl_init_state, tl_states))
        tl_states.drop_duplicates(inplace=True)
        tl_states["duration"] = np.diff(
            tl_states.time_step.values, append=time_steps.max() + 1
        )
        return tl_states

    def __call__(self, window_job: Window, window_meta: SindWindowMeta) -> Scenario:
        meta_scenario = super().__call__(window_job, window_meta)
        time_steps = window_job.vehicle_states.index.get_level_values(-1).unique()
        window_meta.traffic_light_states.time_step = (
            window_meta.traffic_light_states.time_step
        )
        traffic_light_states = self._prepare_traffic_light_df(
            window_meta.traffic_light_states, time_steps
        )
        self._add_traffic_lights(meta_scenario, traffic_light_states)
        return meta_scenario


@dataclass
class SindMetaScenarioGenerator(IMetaScenarioCreator):
    _meta_scenario: Scenario = field(default=None, init=False)

    def __post_init__(self) -> None:
        with importlib.resources.path(
            "commonroad_dataset_converter.datasets.sind", "CHN_SIND-1.xml"
        ) as map_path:
            self._meta_scenario = CommonRoadFileReader(str(map_path)).open()[0]

    def __call__(self, _: Window, window_meta: SindWindowMeta) -> Scenario:
        meta_scenario = copy.deepcopy(self._meta_scenario)
        meta_scenario.scenario_id.map_id = int(
            window_meta.recording_id.replace("_", "")
        )
        return meta_scenario
