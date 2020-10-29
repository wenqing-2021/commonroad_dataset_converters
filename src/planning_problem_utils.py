import random

from commonroad.scenario.trajectory import State
from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.obstacle import ObstacleType


def generate_planning_problem(scenario: Scenario, orientation_half_range: float = 0.2, velocity_half_range: float = 10,
                              time_step_half_range: int = 25, keep_ego: bool = False) -> PlanningProblem:
    """
    Generates planning problem for scenario by taking obstacle trajectory
    :param scenario: CommonRoad scenario
    :param orientation_half_range: parameter for goal state orientation
    :param velocity_half_range: parameter for goal state velocity
    :param time_step_half_range: parameter for goal state time step
    :param keep_ego: boolean indicating if vehicles selected for planning problem should be kept in scenario
    :return: CommonRoad planning problem
    """
    # random choose obstacle as ego vehicle
    random.seed(0)
    dynamic_obstacle_selected = None

    # only choose car type as ego vehicle
    while dynamic_obstacle_selected is None:
        dynamic_obstacle_selected = random.choice(scenario.dynamic_obstacles)
        if dynamic_obstacle_selected.obstacle_type != ObstacleType.CAR:
            dynamic_obstacle_selected = None

    dynamic_obstacle_shape = dynamic_obstacle_selected.obstacle_shape
    dynamic_obstacle_initial_state = dynamic_obstacle_selected.initial_state
    dynamic_obstacle_final_state = dynamic_obstacle_selected.prediction.trajectory.state_list[-1]

    if not keep_ego:
        planning_problem_id = dynamic_obstacle_selected.obstacle_id
        scenario.remove_obstacle(dynamic_obstacle_selected)
    else:
        planning_problem_id = scenario.generate_object_id()

    # define orientation, velocity and time step intervals as goal region
    orientation_interval = AngleInterval(dynamic_obstacle_final_state.orientation - orientation_half_range,
                                         dynamic_obstacle_final_state.orientation + orientation_half_range)
    velocity_interval = Interval(dynamic_obstacle_final_state.velocity - velocity_half_range,
                                 dynamic_obstacle_final_state.velocity + velocity_half_range)

    max_time_step = max([obs.prediction.trajectory.state_list[-1].time_step for obs in scenario.dynamic_obstacles])
    final_time_step = min(dynamic_obstacle_final_state.time_step + time_step_half_range, max_time_step)
    time_step_interval = Interval(0, final_time_step)

    goal_position = Rectangle(dynamic_obstacle_shape.length, dynamic_obstacle_shape.width,
                              center=dynamic_obstacle_final_state.position,
                              orientation=dynamic_obstacle_final_state.orientation)
    goal_region = GoalRegion([State(position=goal_position, orientation=orientation_interval,
                                    velocity=velocity_interval, time_step=time_step_interval)])

    dynamic_obstacle_initial_state.yaw_rate = 0.0
    dynamic_obstacle_initial_state.slip_angle = 0.0

    planning_problem = PlanningProblem(planning_problem_id, dynamic_obstacle_initial_state, goal_region)
    
    return planning_problem
