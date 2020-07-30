from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario, State
from commonroad.common.util import make_valid_orientation, make_valid_orientation_interval


def get_dynamic_obstacle_state(target: DynamicObstacle, time_step: int) -> State:
    """
    Return the state of target object at the time step.
    :param target: Dynamic obstacle, can be other vehicles or ego vehicle
    :param time_step: the desired time step
    :return: state of the obstacle
    """
    if time_step == target.initial_state.time_step:
        state_at_time_step = target.initial_state
    else:
        state_at_time_step = target.prediction.trajectory.state_at_time_step(time_step)
    return state_at_time_step


def get_end_time(scenario: Scenario):
    return max(len(o.prediction.trajectory.state_list) for o in scenario.dynamic_obstacles)


def make_valid_orientation_pruned(orientation: float):
    """
    Make orientation valid and prune to correct representation for XML
    """
    orientation = make_valid_orientation(orientation)
    return max(min(orientation, 6.283185), -6.283185)


def make_valid_orientation_interval_pruned(o1: float, o2: float):
    """
    Make orientation valid and prune to correct representation for XML
    """
    o1, o2 = make_valid_orientation_interval(o1, o2)
    return make_valid_orientation_pruned(o1), make_valid_orientation_pruned(o2)


