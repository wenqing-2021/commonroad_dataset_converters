from dataclasses import dataclass
from pathlib import Path

import pandas as pd
from commonroad.common.file_writer import CommonRoadFileWriter, FileFormat
from commonroad.common.writer.file_writer_interface import OverwriteExistingFile
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import CustomState, InitialState
from commonroad.scenario.trajectory import Trajectory

from ..interface import IScenarioJobConsumer
from ..util.indicator_inference import (
    _add_indicator_lights_based_on_trajectory,
    _generate_empty_signal_series,
)
from .job_producer import TabularJob


@dataclass
class TabularJobConsumer(IScenarioJobConsumer[TabularJob]):
    """Consume conversion jobs of the tabular API."""

    #: Output directory for the converted scenarios.
    output_dir: Path
    #: File format of the converted scenarios.
    file_format: FileFormat = FileFormat.XML
    #: Let the scenario start at time step 0.
    obstacles_start_at_zero: bool = False
    #: Infer turning indicator signal from curvature.
    create_turning_indicator: bool = False

    def _create_scenario(self, job: TabularJob) -> Scenario:
        traj_df = job.vehicle_states.copy()
        # Offset trajectories if required
        traj_df["time_step"] = (
            traj_df.index.get_level_values(-1)
            - traj_df.index.get_level_values(-1).min() * self.obstacles_start_at_zero
        )
        scenario = job.meta_scenario

        idx = pd.IndexSlice
        # Create obstacles
        for track_id, vehicle_meta in job.vehicle_meta.iterrows():
            track_df = traj_df.loc[idx[track_id, :]]
            if len(track_df) < 2:
                continue

            # Create the obstacle
            obs = _create_obstacle(track_df, vehicle_meta, track_id)
            if self.create_turning_indicator:
                # Derive turning indicator from trajectory curvature
                if obs.obstacle_type in [
                    ObstacleType.TAXI,
                    ObstacleType.CAR,
                    ObstacleType.PRIORITY_VEHICLE,
                    ObstacleType.TRUCK,
                    ObstacleType.BUS,
                    ObstacleType.MOTORCYCLE,
                ]:
                    signal_states = _add_indicator_lights_based_on_trajectory(
                        obs.prediction.trajectory,
                        [40, 30],
                        obs.initial_state.time_step,
                        obs.prediction.trajectory.state_list[-1].time_step,
                    )
                else:
                    signal_states = _generate_empty_signal_series(
                        obs.initial_state.time_step,
                        obs.prediction.trajectory.state_list[-1].time_step,
                    )
                obs.initial_signal_state = signal_states[0]
                obs.signal_series = signal_states[1:]
            scenario.add_objects(obs)

        # A scenario is cooperative if it has more than one planning problem
        if len(job.planning_problem_set.planning_problem_dict) > 1:
            scenario.scenario_id.cooperative = True

        return scenario

    def _save_scenario(
        self, scenario: Scenario, planning_problem: PlanningProblemSet
    ) -> None:
        cw = CommonRoadFileWriter(
            scenario, planning_problem, file_format=self.file_format
        )
        # TODO: Can be simplified with next CommonRoad release
        if self.file_format == FileFormat.XML:
            suffix = "xml"
        elif self.file_format == FileFormat.PROTOBUF:
            suffix = "pb"
        else:
            assert False
        cw.write_to_file(
            str(self.output_dir / f"{scenario.scenario_id}.{suffix}"),
            OverwriteExistingFile.ALWAYS,
        )

    def __call__(self, job: TabularJob) -> None:
        scenario = self._create_scenario(job)

        if len(scenario.dynamic_obstacles) == 0:
            # Skip scenario if it is empty.
            return

        # Write scenario file
        self._save_scenario(scenario, job.planning_problem_set)


def _create_obstacle(
    track_df: pd.DataFrame, track_meta: pd.Series, dynamic_obstacle_id: int
) -> DynamicObstacle:
    dynamic_obstacle_type = ObstacleType(track_meta.obstacle_type)
    if dynamic_obstacle_type == ObstacleType.PEDESTRIAN:
        # as surveyed by the author (approximation, harmonized with
        # https://commonroad.in.tum.de/static/scenario_xml/2018b/ZAM_Intersect-1_2_S-1.xml)
        dynamic_obstacle_shape = Circle(0.35)
    elif dynamic_obstacle_type == ObstacleType.BICYCLE:
        # as surveyed by the author (handle_width x bicycle length), harmonized
        # with https://commonroad.in.tum.de/static/scenario_xml/2018b/DEU_Muc-30_1_S-1.xml
        dynamic_obstacle_shape = Rectangle(width=0.6, length=1.8)
    else:
        length = track_meta.length
        width = track_meta.width
        dynamic_obstacle_shape = Rectangle(width=width, length=length)

    state_list = []
    for index, row in track_df.iterrows():
        remaining_row = row.drop(labels=["x", "y", "time_step"])
        state_list.append(
            CustomState(
                time_step=int(row["time_step"]),
                position=row[["x", "y"]].values,
                **remaining_row.to_dict(),
            )
        )

    # dynamic_obstacle_initial_state = state_list[0].convert_state_to_state(InitialState())
    kwargs = {
        f: state_list[0].__dict__[f]
        for f in state_list[0].used_attributes
        if f
        in {
            "time_step",
            "orientation",
            "position",
            "velocity",
            "acceleration",
            "yaw_rate",
            "slip_angle",
        }
    }
    dynamic_obstacle_initial_state = InitialState(**kwargs)

    dynamic_obstacle_trajectory = Trajectory(state_list[1].time_step, state_list[1:])
    dynamic_obstacle_prediction = TrajectoryPrediction(
        dynamic_obstacle_trajectory, dynamic_obstacle_shape
    )

    return DynamicObstacle(
        dynamic_obstacle_id + 10000,
        dynamic_obstacle_type,
        dynamic_obstacle_shape,
        dynamic_obstacle_initial_state,
        dynamic_obstacle_prediction,
    )
