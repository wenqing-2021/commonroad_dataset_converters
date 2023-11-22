from typing import List

import numpy as np
from commonroad.scenario.obstacle import SignalState
from commonroad.scenario.trajectory import Trajectory

from commonroad_dataset_converter.conversion.util.trajectory_classification import (
    TrajectoryType,
    classify_trajectory,
)


def _add_indicator_lights_based_on_trajectory(
    obstacle_trajectory: Trajectory,
    blink_padding: List[int],
    initial_time_step: int,
    final_time_step: int,
    turn_threshold: float = 0.02,
) -> List[SignalState]:
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
    obs_traj, curvature = classify_trajectory(
        obstacle_trajectory, turn_threshold=turn_threshold
    )
    turn_index = [
        np.amin(np.where(curvature == np.amax(curvature))),
        np.amin(np.where(curvature == np.amin(curvature))),
    ]
    signal_states = []
    if obs_traj == TrajectoryType.LEFT or obs_traj == TrajectoryType.RIGHT:
        traj_index = 1 if obs_traj == TrajectoryType.LEFT else 2
        start = max(initial_time_step, turn_index[traj_index - 1] - blink_padding[0])
        end = min(turn_index[traj_index - 1] + blink_padding[1], final_time_step)
        for i in range(initial_time_step, final_time_step + 1):
            turn_left = 1 == traj_index and start <= i <= end
            turn_right = 2 == traj_index and start <= i <= end
            signal_states.append(
                SignalState(
                    time_step=i, indicator_left=turn_left, indicator_right=turn_right
                )
            )
    elif obs_traj == TrajectoryType.BOTH:
        start_left = max(initial_time_step, turn_index[0] - blink_padding[0])
        end_left = min(final_time_step, turn_index[0] + blink_padding[0])
        start_right = max(initial_time_step, turn_index[1] - blink_padding[1])
        end_right = min(final_time_step, turn_index[1] + blink_padding[1])
        for i in range(initial_time_step, final_time_step + 1):
            turn_left = start_left <= i <= end_left
            turn_right = start_right <= i <= end_right
            if turn_right and turn_left:
                turn_left = (
                    end_left > start_right and end_left - i > i - start_right
                ) or (i - start_left >= end_right - i)
                turn_right = not turn_left
            signal_states.append(
                SignalState(
                    time_step=i, indicator_left=turn_left, indicator_right=turn_right
                )
            )
    else:
        return _generate_empty_signal_series(initial_time_step, final_time_step)
    return signal_states


def _generate_empty_signal_series(
    initial_time_step: int, final_time_step: int
) -> List[SignalState]:
    signal_states = []
    for i in range(initial_time_step, final_time_step + 1):
        signal_states.append(
            SignalState(time_step=i, indicator_left=False, indicator_right=False)
        )
    return signal_states
