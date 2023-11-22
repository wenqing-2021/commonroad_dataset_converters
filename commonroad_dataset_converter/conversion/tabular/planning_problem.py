import logging
from dataclasses import dataclass
from typing import Sequence

from commonroad.common.util import AngleInterval, Interval
from commonroad.geometry.shape import Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import CustomState, InitialState

from commonroad_dataset_converter.conversion.tabular.interface import Window
from commonroad_dataset_converter.conversion.util.planning_problem_utils import (
    _cut_lanelet_polygon,
)

_logger = logging.getLogger(__name__)


@dataclass
class EgoWindow(Window):
    ego: Sequence[int]


class PlanningProblemCreator:
    """Create an empty planning problem."""

    def __call__(
        self, window_job: Window, meta_scenario: Scenario
    ) -> PlanningProblemSet:
        return PlanningProblemSet()


@dataclass
class EgoPlanningProblemCreator(PlanningProblemCreator):
    """Create a planning problem for all specified dynamic obstacle ids.

    Planning problems are created by using the initial state of a dynamic obstacle as initial state
    of the planning problem and the final state as the center of the goal region.
    The goal region is an enlarged version of the vehicle's shape.
    Constraints on the goal time step, orientation, and velocity can be specified by interval half ranges.
    """

    #: Whether to keep the original ego vehicle's in the scenario.
    keep_ego: bool
    orientation_half_range: float = 0.2  # rad
    velocity_half_range: float = 10.0  # m/s
    time_step_half_range: int = 25  # 1

    def __call__(
        self, window_job: EgoWindow, meta_scenario: Scenario
    ) -> PlanningProblemSet:
        planning_problem_set = PlanningProblemSet()
        for ego_id in window_job.ego:
            ego_meta = window_job.vehicle_meta.loc[ego_id]
            dynamic_obstacle_shape = Rectangle(ego_meta.length, ego_meta.width)
            dynamic_obstacle_initial_state = window_job.vehicle_states.loc[ego_id].iloc[
                0
            ]
            dynamic_obstacle_final_state = window_job.vehicle_states.loc[ego_id].iloc[
                -1
            ]

            # define orientation, velocity and time step intervals as goal region
            orientation_interval = AngleInterval(
                dynamic_obstacle_final_state.orientation - self.orientation_half_range,
                dynamic_obstacle_final_state.orientation + self.orientation_half_range,
            )
            velocity_interval = Interval(
                dynamic_obstacle_final_state.velocity - self.velocity_half_range,
                dynamic_obstacle_final_state.velocity + self.velocity_half_range,
            )

            final_time_step = min(
                window_job.vehicle_states.loc[ego_id].index.max()
                + self.time_step_half_range,
                window_job.vehicle_states.index.get_level_values(-1).max(),
            )

            time_step_interval = Interval(0, final_time_step)

            goal_shape = Rectangle(
                length=dynamic_obstacle_shape.length + 2.0,
                width=max(dynamic_obstacle_shape.width + 1.0, 3.5),
                center=dynamic_obstacle_final_state[["x", "y"]].values,
                orientation=dynamic_obstacle_final_state.orientation,
            )
            # find goal lanelet
            goal_lanelets = meta_scenario.lanelet_network.find_lanelet_by_position(
                [dynamic_obstacle_final_state[["x", "y"]].values]
            )
            # TODO: Do this check already when selecting ego vehicles
            if len(goal_lanelets[0]) == 0:
                _logger.debug(
                    "Selected final state for planning problem is out of road. Skipping this scenario"
                )
                continue

            goal_position = _cut_lanelet_polygon(
                dynamic_obstacle_final_state[["x", "y"]].values,
                dynamic_obstacle_shape.length + 2.0,
                meta_scenario.lanelet_network,
            )
            if goal_position.shapely_object.area < goal_shape.shapely_object.area:
                goal_position = goal_shape

            goal_region = GoalRegion(
                [
                    CustomState(
                        position=goal_position,
                        orientation=orientation_interval,
                        velocity=velocity_interval,
                        time_step=time_step_interval,
                    )
                ]
            )
            # do not always initialize these attributes
            dynamic_obstacle_initial_state["yaw_rate"] = 0.0
            dynamic_obstacle_initial_state["slip_angle"] = 0.0
            # FIXME
            planning_problem = PlanningProblem(
                ego_id + 100000,
                InitialState(
                    time_step=0,
                    position=dynamic_obstacle_initial_state[["x", "y"]].values,
                    **dynamic_obstacle_initial_state.drop(labels=["x", "y"]),
                ),
                goal_region,
            )
            planning_problem_set.add_planning_problem(planning_problem)
            if not self.keep_ego:
                window_job.vehicle_meta.drop(index=ego_id, inplace=True)

        return planning_problem_set


@dataclass
class RandomObstaclePlanningProblemWrapper(PlanningProblemCreator):
    """Randomly choose dynamic obstacles from a window as a basis for planning problems.

    Only dynamic obstacles of type "car" are considered!
    """

    wrapped_planning_problem_creator: PlanningProblemCreator
    #: Number of planning problems to create per scenario
    num_planning_problems: int = 1

    def __call__(self, window: Window, meta_scenario: Scenario) -> PlanningProblemSet:
        num_states = window.vehicle_states.groupby(level=0).x.count()
        candidates = window.vehicle_meta.loc[
            (num_states > 1) & (window.vehicle_meta.obstacle_type == "car")
        ]
        if len(candidates) >= self.num_planning_problems:
            ego_ids = candidates.sample(self.num_planning_problems).index.values
            ego_window_job = EgoWindow(
                window.vehicle_states, window.vehicle_meta, window.dt, ego_ids
            )
            return self.wrapped_planning_problem_creator(ego_window_job, meta_scenario)
        else:
            return super().__call__(window, meta_scenario)
