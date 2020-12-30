__author__ = "Edmond Irani Liu, Xiao Wang"
__copyright__ = "TUM Cyber-Physical System Group"
__credits__ = [""]
__version__ = "1.0"
__maintainer__ = "Xiao Wang"
__email__ = "xiao.wang@tum.de"
__status__ = ""
# TODO write des
__desc__ = """
TODO
"""

import numpy as np
import pandas as pd

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.prediction.prediction import TrajectoryPrediction


def get_velocity(track_df: pd.DataFrame) -> np.array:
    """
    Calculates velocity given x-velocity and y-velocity

    :param track_df: track data frame of a vehicle
    :return: array of velocities for vehicle
    """
    return np.sqrt(track_df.vx ** 2 + track_df.vy ** 2)

def get_type_obstacle_commonroad(type_agent):
    dict_conversion = {'car': ObstacleType.CAR,
                       'truck': ObstacleType.TRUCK,
                       'bus': ObstacleType.BUS,
                       'bicycle': ObstacleType.BICYCLE,
                       'motorcycle': ObstacleType.MOTORCYCLE}

    type_obstacle_CR = dict_conversion.get(type_agent, ObstacleType.UNKNOWN)

    assert type_obstacle_CR != ObstacleType.UNKNOWN, f"Found obstacle of type Unknown {type_agent}."

    return type_obstacle_CR


def generate_dynamic_obstacle(scenario: Scenario, track_df: pd.DataFrame, time_start_track: int) -> DynamicObstacle:

    length = track_df.length.values[0]
    width = track_df.width.values[0]

    dynamic_obstacle_id = scenario.generate_object_id()
    dynamic_obstacle_type = get_type_obstacle_commonroad(track_df.agent_type.values[0])
    dynamic_obstacle_shape = Rectangle(width=width, length=length)

    xs = np.array(track_df.x)
    ys = np.array(track_df.y)
    velocities = get_velocity(track_df)
    orientations = np.array(track_df.psi_rad)

    state_list = []
    for i, (x, y, v, theta) in enumerate(zip(xs, ys, velocities, orientations)):
        state_list.append(State(position=np.array([x, y]), velocity=v, orientation=theta,
                                time_step=time_start_track + i))

    dynamic_obstacle_initial_state = state_list[0]

    dynamic_obstacle_trajectory = Trajectory(time_start_track + 1, state_list[1:])
    dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, dynamic_obstacle_shape)

    return DynamicObstacle(dynamic_obstacle_id, dynamic_obstacle_type, dynamic_obstacle_shape,
                           dynamic_obstacle_initial_state, dynamic_obstacle_prediction)