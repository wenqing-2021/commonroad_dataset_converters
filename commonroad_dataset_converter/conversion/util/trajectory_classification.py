""" Provides a function to classify a trajectory based on its curvature profile
"""

from enum import Enum
from typing import Iterable, List, Sequence, Tuple

import numpy as np
from commonroad.scenario.trajectory import Trajectory
from scipy.ndimage import uniform_filter1d


class TrajectoryType(Enum):
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    BOTH = 4


def classify_trajectory(
    trajectory: Trajectory, min_velocity: float = 1.0, turn_threshold: float = 0.03
) -> Tuple[TrajectoryType, np.ndarray]:
    """
    Get TrajectoryType of the given trajectory
    :param trajectory: trajectory to classify
    :param min_velocity: curvatures for states with velocity < min_velocity are clamped to 0
    :param turn_threshold: minimum curvature at any point for trajectory to be classified as a turn
    :return: Type of the trajectory and array of calculated curvatures.
    """

    v = [s.velocity for s in trajectory.state_list]
    positions = np.stack([s.position for s in trajectory.state_list])
    c = _calc_curvature(positions)
    c[np.abs(v) < min_velocity] = 0.0
    c = _smooth(c, iterations=2, window=13)

    traj_class = _classify_curvature(c, turn_threshold)

    return traj_class, c


def _classify_curvature(c: np.ndarray, turn_threshold: float) -> TrajectoryType:
    """Get TrajectoryType based off curvatures"""

    min_c = np.min(c)
    max_c = np.max(c)

    is_right_turn = min_c <= -turn_threshold
    is_left_turn = max_c >= turn_threshold

    if is_left_turn and not is_right_turn:
        return TrajectoryType.LEFT
    elif is_right_turn and not is_left_turn:
        return TrajectoryType.RIGHT
    elif not is_left_turn and not is_right_turn:
        return TrajectoryType.STRAIGHT
    else:
        return TrajectoryType.BOTH


def _smooth(x: np.ndarray, iterations: int = 1, window: int = 2) -> np.ndarray:
    if iterations <= 0:
        return x

    x_new = x
    for i in range(iterations):
        x_new = uniform_filter1d(x_new, window * 2 + 1, mode="nearest")

    return x_new


def _calc_curvature(array: np.ndarray) -> np.ndarray:
    if array.shape[0] < 2:
        return np.zeros(array.shape[0])

    dx = np.gradient(array[:, 0])
    dy = np.gradient(array[:, 1])

    velocity = np.column_stack((dx, dy))

    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    acceleration = np.column_stack((ddx, ddy))
    # kappa = a x v / |v|^3
    v_squared_sum = np.sum(velocity**2, axis=-1)
    out = np.zeros_like(v_squared_sum)
    np.divide(
        -np.cross(acceleration, velocity),
        v_squared_sum**1.5,
        where=v_squared_sum > 0.0,
        out=out,
    )

    return out
