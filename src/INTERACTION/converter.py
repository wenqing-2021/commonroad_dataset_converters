import os
import copy
import glob
import numpy as np
import pandas as pd

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile

def generate_planning_problems(scenario, num_problems_desired, id_segment, index, time_end_scenario,
                               time_start_scenario=0, distance_travel_min=30.0, flag_add_goal_position=False,
                               orientation_half_range=0.2, velocity_half_range=5, time_step_half_range=10,
                               position_length=2.0, position_width=2.0, flag_same_direction_problems=False,
                               thresh_distance_coop=25.0, thresh_orient_coop=1.0, flag_verbose=False,
                               num_planning_problems: int = 1, keep_ego: bool = False):
    # final list to hold planning problems
    list_planning_problems = []

    if not len(scenario.dynamic_obstacles):
        if flag_verbose: print(
            f"\t\tScenario with id {scenario.benchmark_id} has no dynamic obstacle, creation of planning problem skipped.")
        return None, list_planning_problems

    obstacles_ordered = []
    for obs in scenario.dynamic_obstacles:
        # we require that the vehicles has moved at least for a certain distance
        if np.linalg.norm(
                obs.initial_state.position - obs.prediction.trajectory.final_state.position) < distance_travel_min:
            continue

        obstacles_ordered.append(obs)

    if not obstacles_ordered:
        return None, list_planning_problems

    # make a reverse list of obstacles base on their duration
    obstacles_ordered.sort(
        key=lambda obs: (obs.prediction.trajectory.final_state.time_step - obs.initial_state.time_step))
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
                            (flag_same_direction_problems and abs(
                                constraint.orientation - obs.initial_state.orientation) > thresh_orient_coop) or \
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
    for i, obs in enumerate(obstacles_chosen):
        if i >= num_planning_problems:
            break
        # TODO maybe num_planning_problems cut off when obstacles get too much
        from src.planning_problem_utils import generate_planning_problem

        if len(scenario.dynamic_obstacles) > (1 if not keep_ego else 0):
            list_planning_problems.append(generate_planning_problem(scenario, orientation_half_range, velocity_half_range,
                                                                    time_step_half_range, keep_ego, obs))

    if len(list_planning_problems) < num_problems_desired:
        if flag_verbose: print(
            f"\t\tDesired number of planning problems for scenario <{scenario.benchmark_id}>: {num_problems_desired}, "
            f"created: {len(list_planning_problems)}")

    return scenario, list_planning_problems



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


def generate_scenarios_without_problems(id_segment, scenario_duration, dt, lanelet_network,
                                        track_df: pd.DataFrame, traffic_signs, obstacle_start_at_zero):
    # time of scenario
    # TODO make function parameters
    time_start_scenario = id_segment * scenario_duration + 1
    time_end_scenario = (id_segment + 1) * (scenario_duration) + 1

    # create CommonRoad scenario object
    scenario = Scenario(dt=dt, scenario_id=None)
    scenario._id_set = set([0])

    # add lanelet network to scenario
    scenario.add_objects(lanelet_network)
    traffic_sign_uses = {}
    for traffic_sign in traffic_signs:
        traffic_sign_uses[traffic_sign.traffic_sign_id] = set()
    for lane in lanelet_network.lanelets:
        for ts in lane.traffic_signs:
            traffic_sign_uses[ts].add(lane.lanelet_id)
    for traffic_sign in traffic_signs:
        if scenario._is_object_id_used(traffic_sign.traffic_sign_id):
            scenario._id_set.remove(traffic_sign.traffic_sign_id)
        scenario.add_objects(traffic_sign, traffic_sign_uses[traffic_sign.traffic_sign_id])



    vehicle_ids = track_df.track_id.unique()

    for id_vehicle in vehicle_ids:
        """
        discard vehicles that (1) start after the scenario ends, or (2) end before the scenario starts.
        for one-shot planning scenarios, we don't consider vehicles that (3) start after time step 0 as well.
        """
        track = track_df[(track_df.track_id == id_vehicle) & (track_df.timestamp_ms >= time_start_scenario)]
        time_start_track = track.timestamp_ms.min()
        time_end_track = track.timestamp_ms.max()

        if len(track) == 0:
            continue

        def enough_time_steps(track: pd.DataFrame):
            if not obstacle_start_at_zero and time_end_scenario - time_start_track < 2 \
                    or time_end_track - time_start_scenario < 2:
                return False
            elif obstacle_start_at_zero and time_start_track > time_start_scenario \
                    or time_end_scenario - time_start_scenario < 2:
                return False
            return True

        if not enough_time_steps(track):
            continue

        time_start_track -= time_start_scenario
        dynamic_obstacle = generate_dynamic_obstacle(scenario, track, int(time_start_track))

        scenario.add_objects(dynamic_obstacle)

    return scenario


def generate_scenario_with_individual_problems(id_segment, id_config_scenario_indi, scenario, scenario_duration,
                                               num_scenario_normal, num_scenario_survival, prefix_name,
                                               directory_output, flag_same_direction_problems, tags,
                                               obstacle_start_at_zero: bool = True,
                                               num_planning_problems: int = 1, keep_ego: bool = False):
    # for some scenarios, we don't add constraints on goal positions to turn them into survival scenarios.
    list_flags_add_goal_position = [True] * num_scenario_normal + [False] * num_scenario_survival

    for idx, flag_add_goal_position in enumerate(list_flags_add_goal_position):
        # if idx > num_planning_problems:
        #     break
        benchmark_id = prefix_name + str(id_config_scenario_indi) + '_T-1'

        scenario_new, list_planning_problems = generate_planning_problems(scenario=scenario,
                                                                          num_problems_desired=1,
                                                                          id_segment=id_segment,
                                                                          index=idx,
                                                                          time_end_scenario=int(scenario_duration * 10),
                                                                          flag_add_goal_position=flag_add_goal_position,
                                                                          flag_same_direction_problems=flag_same_direction_problems,
                                                                          num_planning_problems=num_planning_problems,
                                                                          keep_ego=keep_ego)
        # if no planning problems created
        if not list_planning_problems:
            continue

        scenario_new.scenario_id = benchmark_id
        # create CommonRoad planning problem set object and add planning problems into it
        planning_problem_set = PlanningProblemSet()
        planning_problem_set.add_planning_problem(list_planning_problems[0])

        # create info
        author = 'Xiao Wang'
        affiliation = 'Technical University of Munich, Germany'
        source = 'INTERACTION Dataset'

        # write new scenario
        fw = CommonRoadFileWriter(scenario=scenario_new,
                                  planning_problem_set=planning_problem_set,
                                  author=author,
                                  affiliation=affiliation,
                                  source=source,
                                  tags=tags)
        filename = directory_output + benchmark_id + ".xml"
        fw.write_to_file(filename, OverwriteExistingFile.ALWAYS, check_validity=obstacle_start_at_zero)
        # fw.write_to_file(filename, OverwriteExistingFile.ALWAYS)
        id_config_scenario_indi += 1

    return id_config_scenario_indi


def generate_scenarios_with_cooperative_problems(id_segment, id_config_scenario_coop, scenario, scenario_duration,
                                                 num_scenario_normal, num_scenario_survival,
                                                 prefix_name, directory_output, flag_same_direction_problems, tags,
                                                 obstacle_start_at_zero: bool = True,
                                                 num_planning_problems: int = 1, keep_ego: bool = False):
    # for some scenarios, we don't add constraints on goal positions to turn them into survival scenarios.
    list_flags_add_goal_position = [True] * num_scenario_normal + [False] * num_scenario_survival
    # generate scenarios with 2, 3, 4 and 5 cooperative vehicles
    num_problems_desired = id_segment % 4 + 2

    for idx, flag_add_goal_position in enumerate(list_flags_add_goal_position):
        benchmark_id = "C-" + prefix_name + str(id_config_scenario_coop) + '_T-1'

        scenario_new, list_planning_problems = generate_planning_problems(scenario=scenario,
                                                                          num_problems_desired=num_problems_desired,
                                                                          id_segment=id_segment,
                                                                          index=idx,
                                                                          time_end_scenario=int(scenario_duration * 10),
                                                                          flag_add_goal_position=flag_add_goal_position,
                                                                          flag_same_direction_problems=flag_same_direction_problems,
                                                                          num_planning_problems=num_planning_problems,
                                                                          keep_ego=keep_ego)
        # if one or no planning problems created
        if len(list_planning_problems) <= 1:
            continue

        scenario_new.benchmark_id = benchmark_id
        # create CommonRoad planning problem set object and add planning problems into it
        planning_problem_set = PlanningProblemSet()
        for problem in list_planning_problems:
            planning_problem_set.add_planning_problem(problem)

        # create info
        author = 'Xiao Wang'
        affiliation = 'Technical University of Munich, Germany'
        source = 'INTERACTION Dataset'

        # write new scenario
        fw = CommonRoadFileWriter(scenario=scenario_new,
                                  planning_problem_set=planning_problem_set,
                                  author=author,
                                  affiliation=affiliation,
                                  source=source,
                                  tags=tags)
        filename = directory_output + benchmark_id + ".xml"
        fw.write_to_file(filename, OverwriteExistingFile.ALWAYS, check_validity=obstacle_start_at_zero)
        # fw.write_to_file(filename, OverwriteExistingFile.ALWAYS)
        id_config_scenario_coop += 1

    return id_config_scenario_coop


def generate_scenarios(prefix_name, path_map, directory_data, directory_output, flag_same_direction_problems=False,
                       tags='urban multi_lane oncoming_traffic',
                       x_offset_lanelets=0, y_offset_lanelets=0, x_offset_tracks=0, y_offset_tracks=0,
                       scenario_time_steps=10.0, obstacle_start_at_zero: bool = True,
                       num_planning_problems: int = 1, keep_ego: bool = False, dt=0.1):
    """
    This function creates CommonRoad scenarios out of the INTERACTION dataset.
    """

    # check validity of map file
    assert os.path.isfile(path_map), f"Scenarios with prefix <{prefix_name}> not created. Map file not found."

    # open map and read in scenario and planning problems (empty at the moment)
    scenario_source, _ = CommonRoadFileReader(path_map).open()

    # create output directory
    os.makedirs(directory_output, exist_ok=True)

    # get list of directories in the data directory
    path_files = sorted(glob.glob(os.path.join(directory_data, "*.csv")))
    assert len(path_files), f"Scenarios with prefix <{prefix_name}> not created. Recorded track files not found."

    # this specifies the configuration id of scenario
    id_config_scenario_indi = 1

    # iterate through record files
    # for path_file, name_file in zip(path_files, list_names_files):
    for path_file in path_files:
        track_df = pd.read_csv(path_file, header=0)
        track_df["timestamp_ms"] = (track_df["timestamp_ms"] / 1000. // dt).astype(int)
        time_min = track_df.timestamp_ms.min()
        time_max = track_df.timestamp_ms.max()
        num_segments = int((time_max - time_min) / (scenario_time_steps))

        # translate all positions
        track_df["x"] -= x_offset_tracks
        track_df["y"] -= y_offset_tracks

        # print(f"\tProcessing: {os.path.basename(path_file)}, Start time: {time_min * dt}s, End time: {time_max * dt}s, "
        #       f"Scenario duration: {scenario_time_steps * dt}S, Number of scenario segments: {num_segments}")

        # prepare lanelet network for scenarios from the given source 
        lanelet_network = LaneletNetwork.create_from_lanelet_network(scenario_source.lanelet_network)
        lanelet_network.translate_rotate(np.array([-x_offset_lanelets, -y_offset_lanelets]), 0)

        traffic_signs = scenario_source.lanelet_network.traffic_signs

        for id_segment in range(num_segments):
            # generate scenario of current segment
            scenario_segment = generate_scenarios_without_problems(id_segment, scenario_time_steps, dt, lanelet_network,
                                                                   track_df, traffic_signs, obstacle_start_at_zero)
            # # skip if there is only a few obstacles in the scenario
            # if len(scenario_segment.dynamic_obstacles) < 3:
            #     continue

            # we generate individual and cooperative planning problems separately
            num_scenarios_normal_indi = 2
            num_scenarios_survival_indi = 1

            id_config_scenario_indi = generate_scenario_with_individual_problems(id_segment, id_config_scenario_indi,
                                                                                 copy.copy(scenario_segment),
                                                                                 scenario_time_steps,
                                                                                 num_scenarios_normal_indi,
                                                                                 num_scenarios_survival_indi,
                                                                                 prefix_name, directory_output,
                                                                                 flag_same_direction_problems, tags,
                                                                                 obstacle_start_at_zero,
                                                                                 num_planning_problems,
                                                                                 keep_ego)

            # if (id_segment + 1) % 10 == 0 or (id_segment + 1) == num_segments: print(
            #     f"\t{id_segment + 1} / {num_segments} segments processed.")

    id_config_scenario_indi -= 1

    return id_config_scenario_indi
