__author__ = "Niels Mündler"
__copyright__ = "TUM Cyber-Physical System Group"
__credits__ = [""]
__version__ = "1.0"
__maintainer__ = "Xiao Wang"
__email__ = "xiao.wang@tum.de"
__status__ = "Released"

__desc__ = """
Extracts planning problems from a big recording by removing a dynamic vehicle and replacing its first and last state
with ego vehicle start and goal position
"""

import math
import logging
import numpy as np
import pandas as pd
from typing import Dict, Union, List
from commonroad_dataset_converter.inD import trajectory_classification

from commonroad.scenario.obstacle import (ObstacleType, DynamicObstacle, StaticObstacle, SignalState)
from commonroad.geometry.shape import Rectangle, Circle
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.prediction.prediction import TrajectoryPrediction

from commonroad_dataset_converter.helper import make_valid_orientation_pruned

LOGGER = logging.getLogger(__name__)


def state_from_track_tuple(time_step: int, xcenter: float, ycenter: float, heading: float, lat_velocity: float,
        lon_velocity: float, lat_acceleration: float, lon_acceleration: float, ):
    """
    Convert a tuple of informations (mostly raw from the inD dataset) to a State object
    Description of parameters mostly copied from https://www.ind-dataset.com/format
    The name of the parameter corresponds to the name of the column in the corresponding csv
    :param time_step: The frame for which the information are given. [-]
    :param center: The [x,y] position of the object's centroid in the local coordinate system. [m]
    :param heading: The heading in the local coordinate system. [deg]
    :param lat_velocity: The lateral velocity. 	[m/s]
    :param lon_velocity: The longitudinal velocity. 	[m/s]
    :param lat_acceleration: The lateral acceleration. 	[m/s²]
    :param lon_acceleration: The longitudinal acceleration. 	[m/s²]
    :return:
    """
    return State(time_step=int(time_step), position=np.array([xcenter, ycenter]),
            orientation=make_valid_orientation_pruned(math.radians(heading)), velocity=lon_velocity,
            acceleration=lon_acceleration, )


def generate_obstacle(tracks_df: pd.DataFrame, tracks_meta_df: pd.DataFrame, vehicle_id: int, obstacle_id: int,
        frame_start: int, class_to_type: Dict[str, ObstacleType], detect_static_vehicles=False, ) -> Union[
    StaticObstacle, DynamicObstacle]:
    """
    Converts a single track from a inD dataset recording to a CommonRoad obstacle
    Assumes that the cutting will leave at least 2 frames remaining
    and takes into account whether the traffic participant is parking and which type it has
    :param tracks_df: tracks that contain only the desired frames [frame_start, frame_start + num_steps_scenario]
    :param tracks_meta_df: tracks that contain meta info of the vehicle
    :param vehicle_id: vehicle id in the tracks_meta_df
    :param obstacle_id: unique obstacle id in a CommonRoad Scenario
    :param frame_start: frame start to offset the time steps of an obstacle
    :param class_to_type: mapping from inD types to CommonRoad types
    :param detect_static_vehicles: whether to regard non-moving vehicles as StaticObstacle
    :return: A new Obstacle with unique obstacle ID, Static or Dynamic corresponding to movement in the scenario
    """

    vehicle_meta = tracks_meta_df[tracks_meta_df.trackId == vehicle_id].to_dict(orient="records")[0]
    vehicle_track = tracks_df[tracks_df.trackId == vehicle_id].to_dict(orient="list")

    obstacle_type = ObstacleType(class_to_type[vehicle_meta["class"].lower()])
    # if its VRU (pedestrian or cyclist) the rectangle size is 0 (likely undesireable)
    if obstacle_type == ObstacleType.PEDESTRIAN:
        # as surveyed by the author (approximation, harmonized with
        # https://commonroad.in.tum.de/static/scenario_xml/2018b/ZAM_Intersect-1_2_S-1.xml)
        obstacle_shape = Circle(0.35)
    elif obstacle_type == ObstacleType.BICYCLE:
        # as surveyed by the author (handle_width x bicycle length), harmonized
        # with https://commonroad.in.tum.de/static/scenario_xml/2018b/DEU_Muc-30_1_S-1.xml
        obstacle_shape = Rectangle(width=0.6, length=1.8)
    else:
        obstacle_shape = Rectangle(width=vehicle_meta["width"], length=vehicle_meta["length"])

    # determine if vehicle is parked
    min_x = min(vehicle_track["xCenter"])
    max_x = max(vehicle_track["xCenter"])
    min_y = min(vehicle_track["yCenter"])
    max_y = max(vehicle_track["yCenter"])
    # arbitrary 1 meter threshold: if moved at least one meter, is not a parked vehicle
    # also note that if it disappears before the recording ends or appears after it begins, it is not parked
    if (
            detect_static_vehicles and obstacle_type != ObstacleType.PEDESTRIAN # vehicle moved less than one meter
            # total during recording
            and pow(max_x - min_x, 2) + pow(max_y - min_y, 2) < 1):
        obstacle_type = ObstacleType.PARKED_VEHICLE
        obstacle_initial_state = state_from_track_tuple(time_step=0, xcenter=np.average(vehicle_track["xCenter"]),
                ycenter=np.average(vehicle_track["yCenter"]), heading=np.average(vehicle_track["heading"]),
                lat_velocity=0., lon_velocity=0., lat_acceleration=0., lon_acceleration=0., )

        return StaticObstacle(obstacle_id, obstacle_type, obstacle_shape, obstacle_initial_state)

    track_tuples = zip(np.array(vehicle_track["frame"]) - frame_start, vehicle_track["xCenter"],
            vehicle_track["yCenter"], vehicle_track["heading"], vehicle_track["latVelocity"],
            vehicle_track["lonVelocity"], vehicle_track["latAcceleration"], vehicle_track["lonAcceleration"], )

    obstacle_initial_state = state_from_track_tuple(*next(track_tuples))
    obstacle_state_list = [state_from_track_tuple(*track_tuple) for track_tuple in track_tuples]
    if len(obstacle_state_list) == 0:
        print("f")

    obstacle_trajectory = Trajectory(obstacle_state_list[0].time_step, obstacle_state_list[0:])
    obstacle_trajectory_prediction = TrajectoryPrediction(obstacle_trajectory, obstacle_shape)

    if obstacle_type in [ObstacleType.TAXI, ObstacleType.CAR, ObstacleType.PRIORITY_VEHICLE, ObstacleType.TRUCK,
                         ObstacleType.BUS, ObstacleType.MOTORCYCLE]:
        signal_states = _add_indicator_lights_based_on_trajectory(obstacle_trajectory, [40, 30],
                obstacle_initial_state.time_step, obstacle_state_list[-1].time_step)
    else:
        signal_states = _generate_empty_signal_series(obstacle_initial_state.time_step,
                                                      obstacle_state_list[-1].time_step)
    if signal_states[0].time_step == 0:
        signal_states = signal_states[1:]
    return DynamicObstacle(obstacle_id, obstacle_type, obstacle_shape, obstacle_initial_state,
            obstacle_trajectory_prediction, signal_series=signal_states)


def _add_indicator_lights_based_on_trajectory(obstacle_trajectory: Trajectory, blink_padding: List[int],
                                              initial_time_step: int, final_time_step: int, turn_threshold=0.02) -> \
List[SignalState]:
    """
    Finds the point with the maximum curvature, adds indicator lights to a SignalState object
    according to the curvature of the trajectory.
    :param obstacle_trajectory: trajectory of the obstacle
    :param blink_padding: number of time steps the obstacle keeps the indicator lights on before and after the turn
    stored in blink_padding[0] nad blink_padding[1] respectively
    :param initial_time_step: starting time step
    :param final_time_step: time step up to which the scenario is generated
    :return: a list of signal states with left and right indicators
    """
    obs_traj, curvature = trajectory_classification.classify_trajectory(obstacle_trajectory,
                                                                        turn_threshold=turn_threshold)
    turn_index = [np.amin(np.where(curvature == np.amax(curvature))),
                  np.amin(np.where(curvature == np.amin(curvature)))]
    signal_states = []
    if obs_traj == trajectory_classification.TrajectoryType.LEFT or obs_traj == \
            trajectory_classification.TrajectoryType.RIGHT:
        traj_index = 1 if obs_traj == trajectory_classification.TrajectoryType.LEFT else 2
        start = max(initial_time_step, turn_index[traj_index - 1] - blink_padding[0])
        end = min(turn_index[traj_index - 1] + blink_padding[1], final_time_step)
        for i in range(initial_time_step, final_time_step + 1):
            turn_left = 1 == traj_index and start <= i <= end
            turn_right = 2 == traj_index and start <= i <= end
            signal_states.append(SignalState(time_step=i, indicator_left=turn_left, indicator_right=turn_right))
    elif obs_traj == trajectory_classification.TrajectoryType.BOTH:
        start_left = max(initial_time_step, turn_index[0] - blink_padding[0])
        end_left = min(final_time_step, turn_index[0] + blink_padding[0])
        start_right = max(initial_time_step, turn_index[1] - blink_padding[1])
        end_right = min(final_time_step, turn_index[1] + blink_padding[1])
        for i in range(initial_time_step, final_time_step + 1):
            turn_left = start_left <= i <= end_left
            turn_right = start_right <= i <= end_right
            if turn_right and turn_left:
                turn_left = (end_left > start_right and end_left - i > i - start_right) or (
                            i - start_left >= end_right - i)
                turn_right = not turn_left
            signal_states.append(SignalState(time_step=i, indicator_left=turn_left, indicator_right=turn_right))
    else:
        return _generate_empty_signal_series(initial_time_step, final_time_step)
    return signal_states


def _generate_empty_signal_series(initial_time_step: int, final_time_step: int) -> List[SignalState]:
    signal_states = []
    for i in range(initial_time_step, final_time_step + 1):
        signal_states.append(SignalState(time_step=i, indicator_left=False, indicator_right=False))
    return signal_states
