from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import numpy as np
import pandas as pd
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Location, Scenario, ScenarioID, Tag

from commonroad_dataset_converter.conversion.tabular.interface import (
    IMetaScenarioCreator,
    IRecordingGenerator,
    Window,
)
from commonroad_dataset_converter.conversion.tabular.job_consumer import (
    TabularJobConsumer,
)
from commonroad_dataset_converter.conversion.tabular.windowing import (
    ObstacleWindowGenerator,
)
from commonroad_dataset_converter.datasets.common.levelx_datasets import LevelXRecording
from commonroad_dataset_converter.datasets.common.obstacle_utils import (
    get_acceleration,
    get_orientation,
    get_velocity,
)

from .map_utils import (
    Direction,
    create_highd_lanelet_network,
    get_lane_markings,
    get_speed_limit,
)


@dataclass
class HighdMetaScenarioGenerator(IMetaScenarioCreator):
    num_vertices: int
    road_length: float
    road_offset: float
    shoulder_width: Optional[float] = None
    extend_width: float = 0.0

    def __call__(self, window: Window, window_meta: LevelXRecording) -> Scenario:
        speed_limit = get_speed_limit(window_meta.recording_meta)
        upper_lane_markings, lower_lane_markings = get_lane_markings(
            window_meta.recording_meta, extend_width=self.extend_width
        )

        direction = Direction(window.vehicle_meta["drivingDirection"].iloc[0])

        map_name = (
            window_meta.recording_meta["locationId"]
            + str(direction)
            + str(window_meta.recording_meta["id"])
        )

        scenario_id = ScenarioID(
            country_id="DEU",
            map_name=map_name,
            configuration_id=window_meta.recording_meta["id"],
        )
        meta_scenario = Scenario(
            0.04,
            scenario_id,
            author="fka",
            affiliation="RWTH AAchen",
            source="The HighWay Drone Dataset (highD)",
            tags={Tag.INTERSTATE},
            location=Location(),
        )
        lanelet_network = create_highd_lanelet_network(
            shoulder=self.shoulder_width is not None,
            shoulder_width=self.shoulder_width,
            direction=direction,
            lane_markings=upper_lane_markings
            if direction is Direction.UPPER
            else lower_lane_markings,
            speed_limit=speed_limit,
            road_length=self.road_length,
            road_offset=self.road_offset,
            num_vertices=self.num_vertices,
        )
        meta_scenario.add_objects(lanelet_network)
        return meta_scenario


@dataclass
class HighdRecordingGenerator(IRecordingGenerator):
    data_path: Path
    locations: Dict[int, str]

    def __iter__(self) -> Iterable[Tuple[Window, LevelXRecording]]:
        for tracks_path in self.data_path.rglob("*_tracks.csv"):
            tracks_meta_path = tracks_path.with_name(tracks_path.stem + "Meta.csv")
            recording_meta_path = tracks_path.with_name(
                tracks_path.stem.split("_")[0] + "_recordingMeta.csv"
            )
            vehicle_tracks = pd.read_csv(tracks_path)
            tracks_meta = pd.read_csv(tracks_meta_path)
            recording_meta = pd.read_csv(recording_meta_path)

            vehicle_tracks.rename(
                columns={"width": "length", "height": "width"}, inplace=True
            )
            tracks_meta.rename(
                columns={
                    "width": "length",
                    "height": "width",
                    "class": "obstacle_type",
                },
                inplace=True,
            )
            tracks_meta["obstacle_type"] = tracks_meta["obstacle_type"].str.lower()

            vehicle_tracks.x = np.array(vehicle_tracks.x + vehicle_tracks.length / 2)
            vehicle_tracks.y = np.array(-(vehicle_tracks.y + vehicle_tracks.width / 2))
            vehicle_tracks["velocity"] = get_velocity(vehicle_tracks)
            vehicle_tracks["orientation"] = get_orientation(vehicle_tracks)
            vehicle_tracks["acceleration"] = get_acceleration(
                vehicle_tracks, vehicle_tracks.orientation
            )

            recording_meta.locationId.replace(self.locations, inplace=True)

            vehicle_tracks = pd.merge(
                vehicle_tracks, tracks_meta[["id", "drivingDirection"]], on="id"
            )
            vehicle_tracks.set_index(["id", "frame"], inplace=True)

            tracks_meta.set_index(["id"], inplace=True)
            vehicle_tracks_upper = vehicle_tracks[vehicle_tracks.drivingDirection == 1]
            vehicle_meta_upper = tracks_meta[tracks_meta.drivingDirection == 1]
            vehicle_tracks_lower = vehicle_tracks[vehicle_tracks.drivingDirection == 2]
            vehicle_meta_lower = tracks_meta[tracks_meta.drivingDirection == 2]

            yield Window(
                vehicle_tracks_upper[
                    ["x", "y", "velocity", "orientation", "acceleration"]
                ],
                vehicle_meta_upper,
                0.04,
            ), LevelXRecording(recording_meta.iloc[0].to_dict())
            yield Window(
                vehicle_tracks_lower[
                    ["x", "y", "velocity", "orientation", "acceleration"]
                ],
                vehicle_meta_lower,
                0.04,
            ), LevelXRecording(recording_meta.iloc[0].to_dict())


@dataclass
class HigdTabularJobProcessor(TabularJobConsumer):
    keep_direction: bool = True

    def _save_scenario(self, scenario: Scenario, planning_problem: PlanningProblemSet):
        if not self.keep_direction and "Upper" in scenario.scenario_id.map_name:
            scenario.translate_rotate(np.zeros(2), np.pi)
            planning_problem.translate_rotate(np.zeros(2), np.pi)
        super()._save_scenario(scenario, planning_problem)


@dataclass
class HighdObstacleWindowGenerator(ObstacleWindowGenerator):
    lane_change: bool = False

    def _get_index(self, recording: Window) -> pd.Index:
        index = super()._get_index(recording)
        if self.lane_change:
            meta = recording.vehicle_meta.loc[index]
            index = meta[meta["numLaneChanges"] > 0].index
        return index
