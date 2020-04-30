import random
import numpy as np

from commonroad.scenario.trajectory import State
from commonroad.common.util import Interval, AngleInterval
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.planning.goal import GoalRegion
from commonroad.geometry.shape import Rectangle


def generate_planning_problems(scenario, num_problems_desired, id_segment, index, time_end_scenario,
                               time_start_scenario=0, distance_travel_min=30.0, flag_add_goal_position=False,
                               orientation_half_range=0.2, velocity_half_range=5,time_step_half_range=10,
                               position_length=2.0, position_width=2.0,
                               flag_same_direction_problems=False, thresh_distance_coop=25.0, thresh_orient_coop=1.0, flag_verbose=False):

    # final list to hold planning problems
    list_planning_problems = []

    if not len(scenario.dynamic_obstacles):
        if flag_verbose: print(f"\t\tScenario with id {scenario.benchmark_id} has no dynamic obstacle, creation of planning problem skipped.")
        return None, list_planning_problems
    
    obstacles_ordered = []
    for obs in scenario.dynamic_obstacles:
        # we require that the vehicles has moved at least for a certain distance
        if np.linalg.norm(obs.initial_state.position - obs.prediction.trajectory.final_state.position) < distance_travel_min:
            continue
        
        obstacles_ordered.append(obs)
    
    if not obstacles_ordered:
        return None, list_planning_problems

    # make a reverse list of obstacles base on their duration
    obstacles_ordered.sort(key=lambda obs: (obs.prediction.trajectory.final_state.time_step - obs.initial_state.time_step))
    obstacles_ordered.reverse()

    # some candidate indices to retrieve obstacles to form diverse scenarios
    list_idx_retrieve = [(id_segment + index * 2) % 20, 
                         (id_segment + index) % 20, 
                         (index * 2) % 20, 
                          index % 20, 
                          0]

    obstacles_candidate = []
    if num_problems_desired == 1:
        # individual planning problem
        # try retrieving obstacles using the list of indices
        for idx in list_idx_retrieve:
            try:
                obstacles_candidate.append(obstacles_ordered[idx])
            except IndexError:
                continue
            else:
                break
    else:
        # cooperative planning
        # try retrieving obstacles using the list of indices
        for idx in list_idx_retrieve:
            try:
                obstacles_candidate.append(obstacles_ordered[idx])
            except IndexError:
                continue
            else:
                constraint = obstacles_candidate[0].initial_state
                id_constraint = obstacles_candidate[0].obstacle_id
                # try adding companions for this poor guy
                for obs in obstacles_ordered:
                    # ensure all proposed problems for cooperative scenarios start near to each other. if required, going in the same direction as well
                    # skip if (1) the distance of current obstacle to constraint is over a threshold, 
                    # or if (2) they have different driving direction (if examined)
                    if np.linalg.norm(constraint.position - obs.initial_state.position) > thresh_distance_coop or \
                       (flag_same_direction_problems and abs(constraint.orientation - obs.initial_state.orientation) > thresh_orient_coop) or \
                        obs.obstacle_id == id_constraint:
                        continue
                    else:
                        obstacles_candidate.append(obs)

                break
        
    # now we choose final planning problems from the candidate ones
    obstacles_chosen = []
    
    if num_problems_desired == 1:
        obstacles_chosen.append(obstacles_candidate[0])
    else:
        # if less than 2 candidate obstacles are present
        if len(obstacles_candidate) < 2:
            return None, list_planning_problems

        # distribute generated problems evenly
        index_offset = (len(obstacles_candidate) - 1) // (num_problems_desired - 1)
        if not index_offset:
            index_offset = 1

        for i in range(0, len(obstacles_candidate), index_offset):
            obstacles_chosen.append(obstacles_candidate[i])
    
    # turn these obstacles into planning problems
    for obs in obstacles_chosen:
        state_final_obstacle = obs.prediction.trajectory.final_state
        scenario.remove_obstacle(obs)
        
        # define orientation, velocity and time step intervals as part of the goal region
        interval_orientation = AngleInterval(state_final_obstacle.orientation - orientation_half_range, 
                                            state_final_obstacle.orientation + orientation_half_range)

        interval_velocity = Interval(max(state_final_obstacle.velocity - velocity_half_range, 0), 
                                    state_final_obstacle.velocity + velocity_half_range)

        interval_time_step = Interval(max(state_final_obstacle.time_step - time_step_half_range, 0),
                                    state_final_obstacle.time_step + time_step_half_range)

        dict_keywords = {'orientation': interval_orientation,
                         'velocity': interval_velocity,
                         'time_step': interval_time_step}

        if flag_add_goal_position:
            position_area = Rectangle(center=state_final_obstacle.position, 
                                      length=position_length, width=position_width, 
                                      orientation=state_final_obstacle.orientation)
            dict_keywords['position'] = position_area

        goal_region = GoalRegion([State(**dict_keywords)])
        
        # add info for initial state
        obs.initial_state.yaw_rate = 0
        obs.initial_state.slip_angle = 0

        # add planning problem to list
        list_planning_problems.append(PlanningProblem(obs.obstacle_id, obs.initial_state, goal_region))

    if len(list_planning_problems) < num_problems_desired:
        if flag_verbose: print(f"\t\tDesired number of planning problems for scenario <{scenario.benchmark_id}>: {num_problems_desired}, created: {len(list_planning_problems)}")
        
    return scenario, list_planning_problems