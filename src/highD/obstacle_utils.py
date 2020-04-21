import scipy.signal
import numpy as np
from typing import Union
from pandas import DataFrame, Series

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import State
from commonroad.scenario.trajectory import Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.scenario import Scenario


obstacle_class_dict = {
    'Truck': ObstacleType.TRUCK,
    'Car': ObstacleType.CAR
}


def savgol_filter(signal: Series, poly_order: int =7) -> Union[np.array, None]:
    """

    :param signal: signal to be filtered
    :param poly_order: The order of the polynomial used to fit the samples
    :return: filtered signal
    """
    window_size = len(signal) // 4
    if window_size % 2 == 0:
        window_size -= 1
    if window_size <= poly_order:
        return None
    else:
        return scipy.signal.savgol_filter(signal, window_size, polyorder=poly_order)


def filter_signal(signal: Series, poly_order: Union[int, None] = None) -> np.array:
    """

    :param signal: signal to be filtered
    :param popoly_orderlyorder: order of the polynomial used to fit the samples
    :return: filtered signal or None
    """
    if poly_order is None:  # no filter
        return np.array(signal)
    else:
        # apply savgol filter with polyorder
        return savgol_filter(signal, poly_order=poly_order)


def get_velocity(track_df: DataFrame) -> np.array:
    """
    Calculates velocity given x-velocity and y-velocity

    :param track_df: track data frame of a vehicle
    :return: array of velocities for vehicle
    """
    return np.sqrt(track_df.xVelocity**2 + track_df.yVelocity**2)


def get_orientation(track_df: DataFrame) -> np.array:
    """
    Calculates orientation given x-velocity and y-velocity

    :param track_df: track data frame of a vehicle
    :return: array of orientations for vehicle
    """
    return np.arctan2(-track_df.yVelocity, track_df.xVelocity)


def get_acceleration(track_df: DataFrame) -> np.array:
    """
    Calculates acceleration given x-acceleration and y-acceleration

    :param track_df: track data frame of a vehicle
    :return: array of accelerations for vehicle
    """
    return np.sqrt(track_df.xAcceleration**2 + track_df.yAcceleration**2)


def generate_dynamic_obstacle(scenario: Scenario, vehicle_id: int, tracks_meta_df: DataFrame,
                              tracks_df: DataFrame, time_step_correction: int) -> DynamicObstacle:
    """

    :param scenario: CommonRoad scenario
    :param vehicle_id: ID of obstacle to generate
    :param tracks_meta_df: track meta information data frames
    :param tracks_df: track data frames
    :return: CommonRoad dynamic obstacle
    """
    
    vehicle_meta = tracks_meta_df[tracks_meta_df.id == vehicle_id]
    vehicle_tracks = tracks_df[tracks_df.id == vehicle_id]

    length = vehicle_meta.width.values[0]
    width = vehicle_meta.height.values[0]

    initial_time_step = int(vehicle_tracks.frame.values[0]) - time_step_correction
    dynamic_obstacle_id = scenario.generate_object_id()
    dynamic_obstacle_type = obstacle_class_dict[vehicle_meta['class'].values[0]]
    dynamic_obstacle_shape = Rectangle(width=width, length=length)

    poly_order = None
    xs = filter_signal(vehicle_tracks.x, poly_order=None)  # checked x signals, no need to filter
    ys = filter_signal(-vehicle_tracks.y, poly_order=poly_order)
    velocities = filter_signal(get_velocity(vehicle_tracks), poly_order=poly_order)
    orientations = filter_signal(get_orientation(vehicle_tracks), poly_order=poly_order)
    accelerations = filter_signal(get_acceleration(vehicle_tracks), poly_order=poly_order)

    state_list = []
    for i, (x, y, v, theta, a) in enumerate(zip(xs, ys, velocities, orientations, accelerations)):
        state_list.append(State(position=np.array([x, y]), velocity=v, orientation=theta,
                                time_step=initial_time_step+i))

    dynamic_obstacle_initial_state = state_list[0]

    dynamic_obstacle_trajectory = Trajectory(initial_time_step+1, state_list[1:])
    dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, dynamic_obstacle_shape)

    return DynamicObstacle(dynamic_obstacle_id, dynamic_obstacle_type, dynamic_obstacle_shape,
                           dynamic_obstacle_initial_state, dynamic_obstacle_prediction)
