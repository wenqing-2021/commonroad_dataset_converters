__author__ = "Niels Mündler, Xiao Wang, Michael Feil"
__copyright__ = "TUM Cyber-Physical System Group"
__credits__ = [""]
__version__ = "1.0"
__maintainer__ = "Xiao Wang"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"

__desc__ = """
    Utilities for creating planning problems in the Converters
"""

import random
from enum import Enum
from typing import Type
import numpy as np
import warnings
import sys

from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.trajectory import State
from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle

try:
    from commonroad_route_planner.route_planner import RoutePlanner
except ModuleNotFoundError as exp:
    RoutePlanner = None
    warnings.warn("module commonroad_route_planner not installed, routability check will be skipped.")


class NoCarException(Exception):
    pass


class Routability(Enum):
    ANY = 0
    # REGULAR_ANDREVERSED = 1
    REGULAR_STRICT = 2

    @classmethod
    def options(cls):
        return [type(item) for item in cls]


def obstacle_to_planning_problem(obstacle: DynamicObstacle, planning_problem_id: int, final_time_step=None,
                                 orientation_half_range: float = 0.2,
                                 velocity_half_range: float = 10, time_step_half_range: int = 25,
                                 lanelet_network: LaneletNetwork = None, highD: bool = False):
    """
    Generates planning problem using initial and final states of a DynamicObstacle
    """
    dynamic_obstacle_shape = obstacle.obstacle_shape
    dynamic_obstacle_initial_state = obstacle.initial_state
    dynamic_obstacle_final_state = obstacle.prediction.trajectory.final_state

    # define orientation, velocity and time step intervals as goal region
    orientation_interval = AngleInterval(dynamic_obstacle_final_state.orientation - orientation_half_range,
                                         dynamic_obstacle_final_state.orientation + orientation_half_range)
    velocity_interval = Interval(dynamic_obstacle_final_state.velocity - velocity_half_range,
                                 dynamic_obstacle_final_state.velocity + velocity_half_range)
    if final_time_step is None:
        final_time_step = dynamic_obstacle_final_state.time_step + time_step_half_range

    time_step_interval = Interval(0, final_time_step)

    goal_shape = Rectangle(length=dynamic_obstacle_shape.length + 2.0,
                           width=max(dynamic_obstacle_shape.width + 1.0, 3.5),
                           center=dynamic_obstacle_final_state.position,
                           orientation=dynamic_obstacle_final_state.orientation)
    # find goal lanelet # TODO: goal lanelet is too large for highD
    goal_lanelets = lanelet_network.find_lanelet_by_position([dynamic_obstacle_final_state.position])
    if len(goal_lanelets[0]) == 0:
        raise NoCarException("Selected final state for planning problem is out of road. Skipping this scenario")

    goal_lanelet_id = goal_lanelets[0][0]
    goal_lanelet = lanelet_network.find_lanelet_by_id(goal_lanelet_id)
    if not highD:
        goal_lanelet_polygon = goal_lanelet.convert_to_polygon()
        if goal_lanelet_polygon.shapely_object.area > goal_shape.shapely_object.area:
            goal_position = goal_lanelet_polygon
        else:
            goal_position = goal_shape
    else:
        # works only for highD because of constant lane width
        lanelet_width = abs(goal_lanelet.left_vertices[-1][1] - goal_lanelet.right_vertices[-1][1])
        goal_position = Rectangle(length=20,
                                  width=lanelet_width,
                                  center=dynamic_obstacle_final_state.position,
                                  orientation=0.)

    goal_region = GoalRegion([State(position=goal_position, orientation=orientation_interval,
                                    velocity=velocity_interval, time_step=time_step_interval)])

    dynamic_obstacle_initial_state.yaw_rate = 0.0
    dynamic_obstacle_initial_state.slip_angle = 0.0

    return PlanningProblem(planning_problem_id, dynamic_obstacle_initial_state, goal_region)


def generate_planning_problem(scenario: Scenario, orientation_half_range: float = 0.2, velocity_half_range: float = 10.,
                              time_step_half_range: int = 25, keep_ego: bool = False, highD: bool = False,
                              lane_change: bool = False, routability: Routability = Routability.ANY) -> PlanningProblem:
    """
    Generates planning problem for scenario by taking obstacle trajectory
    :param scenario: CommonRoad scenario
    :param orientation_half_range: parameter for goal state orientation
    :param velocity_half_range: parameter for goal state velocity
    :param time_step_half_range: parameter for goal state time step
    :param keep_ego: boolean indicating if vehicles selected for planning problem should be kept in scenario
    :param highD: indicator for highD dataset; select only vehicles that drive to the end of the road
    :param lane_change (only for highD): select only vehicles that located on different lanelets at initial and final ts
    :return: CommonRoad planning problem
    """
    # random choose obstacle as ego vehicle
    random.seed(0)

    # if len(scenario.dynamic_obstacles) == 0: # TODO: duplicated?
    #     raise NoCarException("There is no car in dynamic obstacles which can be used as planning problem.")

    # only choose car type as ego vehicle
    car_obstacles = [obstacle for obstacle in scenario.dynamic_obstacles if (obstacle.obstacle_type == ObstacleType.CAR
                                                                             and obstacle.initial_state.time_step == 0
                                                                             )]
    # select only vehicles that drive to the end of the road
    if highD:
        car_obstacles_highD = [obs for obs in car_obstacles
                               if abs(obs.initial_state.position[0] -
                                      obs.state_at_time(obs.prediction.final_time_step).position[0]) > 100.]
        if lane_change:
            car_lane_changing = []
            lanelet_network = scenario.lanelet_network
            for obs in car_obstacles_highD:
                initial_lanelet = lanelet_network.find_lanelet_by_position([obs.initial_state.position])[0][0]
                final_lanelet = lanelet_network.find_lanelet_by_position([
                    obs.state_at_time(obs.prediction.final_time_step).position])[0][0]
                if initial_lanelet != final_lanelet:
                    car_lane_changing.append(obs)
            if len(car_lane_changing) == 0:
                warnings.warn("No lane changing vehicle available, using lane keeping vehicle as planning problem")
            else:
                car_obstacles_highD = car_lane_changing

        car_obstacles = car_obstacles_highD

    random.shuffle(car_obstacles)

    while True:
        if len(car_obstacles) > 0:
            dynamic_obstacle_selected = car_obstacles.pop()
        else:
            raise NoCarException("There is no car in dynamic obstacles which can be used as planning problem.")

        # check validity of dynamic_obstacle_selected
        if not obstacle_moved(dynamic_obstacle_selected):
            continue

        if not keep_ego:
            planning_problem_id = dynamic_obstacle_selected.obstacle_id
        else:
            planning_problem_id = scenario.generate_object_id()

        if len(scenario.dynamic_obstacles) > 0:
            max_time_step = max([obstacle.prediction.final_time_step for obstacle in scenario.dynamic_obstacles])
            final_time_step = min(
                dynamic_obstacle_selected.prediction.trajectory.final_state.time_step + time_step_half_range, max_time_step)
        else:
            final_time_step = dynamic_obstacle_selected.prediction.trajectory.final_state.time_step + time_step_half_range

        planning_problem = obstacle_to_planning_problem(dynamic_obstacle_selected,
                                                        planning_problem_id,
                                                        final_time_step=final_time_step,
                                                        orientation_half_range=orientation_half_range,
                                                        velocity_half_range=velocity_half_range,
                                                        time_step_half_range=time_step_half_range,
                                                        lanelet_network=scenario.lanelet_network,
                                                        highD=highD)

        # check if generated planning problem is routable
        if routability == Routability.ANY or check_routability_planning_problem(scenario, planning_problem, routability):
            if not keep_ego:
                scenario.remove_obstacle(dynamic_obstacle_selected)
            return planning_problem


def obstacle_moved(obstacle):
    # positions = np.array([state.position for state in obstacle.prediction.trajectory.state_list])
    # min_x = np.min(positions[:, 0])
    # max_x = np.min(positions[:, 0])
    # min_y = np.min(positions[:, 1])
    # max_y = np.min(positions[:, 1])
    #
    # tmp1 = pow(max_x - min_x, 2) + pow(max_y - min_y, 2)

    driven_distance = 0.
    occupancy_set_index_div_5 = (len(obstacle.prediction.occupancy_set) - 1) // 5
    for i in range(occupancy_set_index_div_5):
        driven_distance += np.abs(np.linalg.norm(obstacle.prediction.trajectory.state_list[i * 5].position
                                                 - obstacle.prediction.trajectory.state_list[
                                                     (i + 1) * 5].position))

    if len(obstacle.prediction.occupancy_set) % 5 != 0:
        driven_distance += np.abs(np.linalg.norm(obstacle.prediction.trajectory.
                                                 state_list[occupancy_set_index_div_5 * 5].position -
                                                 obstacle.prediction.trajectory.final_state.position))

    # discard candidate if driven distance in scenario is too short
    return driven_distance > 5.


def check_routability_planning_problem(
    scenario: Scenario, planning_problem: PlanningProblem,
    max_difficulity: Type[Routability]
) -> bool:
    """
    Checks if a planning problem is routable on scenario
    :param scenario: CommonRoad scenario
    :param planning_problem: Planning Problem to be solved
    :param max_difficulity: difficulty until which planing problem is considered routable.
        Routability.ANY: dont do any checks, always return True
        Routability.REGULAR_STRICT: only return True if default route planner can find a route

    :return: bool, True if CommonRoad planning problem is routeable with max_difficulity
    """
    if max_difficulity == Routability.ANY or RoutePlanner is None:
        return True

    elif max_difficulity == Routability.REGULAR_STRICT:
        route_planner = RoutePlanner(scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)
        candidate_holder = route_planner.plan_routes()
        _, num_candiadates = candidate_holder.retrieve_all_routes()

        if num_candiadates > 0:
            return True  # there are some routes.
        else:
            return False

    else:
        warnings.warn(f"option not defined: {max_difficulity}")
