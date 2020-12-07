import os
import csv
import copy
import numpy as np

from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.obstacle import DynamicObstacle, StaticObstacle, ObstacleType
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.common.util import Interval
from commonroad.geometry.shape import Rectangle, Circle
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile

from src.INTERACTION.src.utils import generate_planning_problems

def create_dictionary_track(path_file, x_offset_tracks, y_offset_tracks):
    # read in from csv file
    file_csv = open(path_file)
    reader_csv = csv.DictReader(file_csv)

    # store tracks info in a dictionary
    dict_tracks = {}
    for row in reader_csv:
        id_vehicle = row['track_id']
        id_frame = row['frame_id']
        time_step = float(row['timestamp_ms']) / 1000
        type_agent = row['agent_type']

        # translate everything so the origin is close to 0
        pos_x = float(row['x']) - x_offset_tracks
        pos_y = float(row['y']) - y_offset_tracks
        v_x = float(row['vx'])
        v_y = float(row['vy'])
        angle = float(row['psi_rad'])
        length = float(row['length'])
        width = float(row['width'])

        try:
            dict_tracks[id_vehicle]['info_track'].append(
                {'time_step': time_step, 'x': pos_x, 'y': pos_y, 'v_x': v_x, 'v_y': v_y, 'angle': angle})
        except KeyError:
            # if no data has been created for this track id, add a new entry to the dictionary
            dict_tracks[id_vehicle] = {'type_agent': type_agent, 'length': length, 'width': width, 'info_track': []}
            dict_tracks[id_vehicle]['info_track'].append(
                {'time_step': time_step, 'x': pos_x, 'y': pos_y, 'v_x': v_x, 'v_y': v_y, 'angle': angle})

    return dict_tracks


def parse_dictionary_tracks(dict_tracks, scenario_duration):
    # get minimum and maximum time of a recording base on the track info
    list_time_track = []
    for id_vehicle in dict_tracks:
        list_time_track.append(dict_tracks[id_vehicle]['info_track'][0]['time_step'])
        list_time_track.append(dict_tracks[id_vehicle]['info_track'][-1]['time_step'])
    time_min, time_max = min(list_time_track), max(list_time_track)

    # compute number of scenarios to be created
    num_segments = int((time_max - time_min) / (scenario_duration))

    return time_min, time_max, num_segments


def get_type_obstacle_commonroad(type_agent):
    dict_conversion = {'car': ObstacleType.CAR,
                       'truck': ObstacleType.TRUCK,
                       'bus': ObstacleType.BUS,
                       'bicycle': ObstacleType.BICYCLE}

    type_obstacle_CR = dict_conversion.get(type_agent, ObstacleType.UNKNOWN)

    assert type_obstacle_CR != ObstacleType.UNKNOWN, "Found obstacle of type Unknown."

    return type_obstacle_CR


def generate_scenarios_without_problems(id_segment, scenario_duration, dt, lanelet_network, dict_tracks, trafficSigns):
    # time of scenario
    # TODO make function parameters
    time_start_scenario = id_segment * scenario_duration + 1
    time_end_scenario = (id_segment + 1) * (scenario_duration) + 1

    # create CommonRoad scenario object
    scenario = Scenario(dt=dt, benchmark_id='')
    scenario._id_set = set([0])

    # add lanelet network to scenario
    scenario.add_objects(lanelet_network)
    traffic_sign_uses = {}
    for traffic_sign in trafficSigns:
        traffic_sign_uses[traffic_sign.traffic_sign_id] = set()
    for lane in lanelet_network.lanelets:
        for ts in lane.traffic_signs:
            traffic_sign_uses[ts].add(lane.lanelet_id)
    for traffic_sign in trafficSigns:
        scenario.add_objects(traffic_sign, traffic_sign_uses[traffic_sign.traffic_sign_id])

    for id_vehicle in dict_tracks:
        """
        discard vehicles that (1) start after the scenario ends, or (2) end before the scenario starts.
        for one-shot planning scenarios, we don't consider vehicles that (3) start after time step 0 as well.
        """
        track = dict_tracks[id_vehicle]
        time_start_track = track['info_track'][0]['time_step']
        time_end_track = track['info_track'][-1]['time_step']

        if (time_start_track >= time_end_scenario or \
                time_end_track <= time_start_scenario or \
                time_start_track >= time_start_scenario):
            continue

        type_obstacle = get_type_obstacle_commonroad(track['type_agent'])
        id_obstacle = scenario.generate_object_id()
        length = track['length']
        width = track['width']
        shape = Rectangle(length, width)
        state_list = []

        for step_info in track['info_track']:
            time_step_track = step_info['time_step']

            # if this timestep happens before scenario starts, continue looking at the track until we find a timestep in range
            if (time_step_track < time_start_scenario):
                continue
            # if this timestep happens after scenario ends, stop looking at this track since the rest occurs later
            if (time_step_track > time_end_scenario):
                break

            # get or compute attributes
            position = np.array([step_info['x'], step_info['y']])
            orientation = step_info['angle']
            vx = step_info['v_x']
            vy = step_info['v_y']
            velocity = np.sqrt(vx ** 2 + vy ** 2)

            # scenario start sets an offset for the timesteps recorded (time should be relative to scenario start)
            time_step_track -= time_start_scenario
            timestep = int(round(time_step_track / dt))
            new_state = State(position=position,
                              orientation=orientation,
                              velocity=velocity,
                              time_step=timestep)
            state_list.append(new_state)

        # omit this track if it has less than two states
        if len(state_list) < 2:
            continue

        # create the predicted trajectory starting at second state in state_list
        trajectory = Trajectory(initial_time_step=state_list[1].time_step, state_list=state_list[1:])
        # create prediction object using the predicted trajectory and the shape of the obstacle
        prediction = TrajectoryPrediction(trajectory=trajectory, shape=shape)

        # generate the dynamic obstacle according to the specification
        obstacle = DynamicObstacle(obstacle_id=id_obstacle,
                                   obstacle_type=type_obstacle,
                                   obstacle_shape=shape,
                                   initial_state=state_list[0],
                                   prediction=prediction)

        # add obstacle to the scenario
        scenario.add_objects(obstacle)

    return scenario


def generate_scenario_with_individual_problems(id_segment, id_config_scenario_indi, scenario, scenario_duration,
                                               num_scenario_normal, num_scenario_survival, prefix_name,
                                               directory_output, flag_same_direction_problems, tags,
                                               obstacle_start_at_zero: bool = True,
                                               num_planning_problems: int =1, keep_ego:bool = False):
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

        scenario_new.benchmark_id = benchmark_id
        # create CommonRoad planning problem set object and add planning problems into it
        planning_problem_set = PlanningProblemSet()
        planning_problem_set.add_planning_problem(list_planning_problems[0])

        # create info
        author = 'Edmond Irani Liu'
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
        fw.write_to_file(filename, OverwriteExistingFile.ALWAYS,check_validity=obstacle_start_at_zero)
        # fw.write_to_file(filename, OverwriteExistingFile.ALWAYS)
        id_config_scenario_indi += 1

    return id_config_scenario_indi


def generate_scenarios_with_cooperative_problems(id_segment, id_config_scenario_coop, scenario, scenario_duration,
                                                 num_scenario_normal, num_scenario_survival,
                                                 prefix_name, directory_output, flag_same_direction_problems, tags,
                                                 obstacle_start_at_zero:bool = True,
                                                 num_planning_problems: int =1, keep_ego:bool = False):
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
        author = 'Edmond Irani Liu'
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
        fw.write_to_file(filename, OverwriteExistingFile.ALWAYS,check_validity=obstacle_start_at_zero)
        # fw.write_to_file(filename, OverwriteExistingFile.ALWAYS)
        id_config_scenario_coop += 1

    return id_config_scenario_coop


def generate_scenarios(prefix_name, path_map, directory_data, directory_output, flag_same_direction_problems=False,
                       tags='urban multi_lane oncoming_traffic',
                       x_offset_lanelets=0, y_offset_lanelets=0, x_offset_tracks=0, y_offset_tracks=0,
                       scenario_duration=10.0, obstacle_start_at_zero: bool = True,
                       num_planning_problems: int = 1, keep_ego: bool = False, dt=0.1):
    """
    This function creates CommonRoad scenarios out of the INTERACTION dataset.
    """

    # check validity of map file
    assert os.path.isfile(path_map), f"Scenarios with prefix <{prefix_name}> not created. Map file not found."
    # open map and read in scenario and planning problems (empty at the moment)
    scenario_source, _ = CommonRoadFileReader(path_map).open()

    # create output directory
    if not os.path.exists(directory_output):
        os.makedirs(directory_output)

        # get list of directories in the data directory
    list_names_files = os.listdir(directory_data)
    num_files = len(list_names_files)
    assert num_files, f"Scenarios with prefix <{prefix_name}> not created. Recorded track files not found."

    path_files = [directory_data + name_file for name_file in list_names_files]

    # this specifies the configuration id of scenario
    id_config_scenario_indi = 1
    id_config_scenario_coop = 1

    # iterate through record files
    for path_file, name_file in zip(path_files, list_names_files):
        dict_tracks = create_dictionary_track(path_file, x_offset_tracks, y_offset_tracks)

        time_min, time_max, num_segments = parse_dictionary_tracks(dict_tracks, scenario_duration)
        print(
            f"\tProcessing: {name_file}, Start time: {time_min}s, End time: {time_max}s, Scenario duration: {scenario_duration}s, Number of scenario segments: {num_segments}")

        # prepare lanelet network for scenarios from the given source 
        lanelet_network = LaneletNetwork.create_from_lanelet_network(scenario_source.lanelet_network)
        lanelet_network.translate_rotate(np.array([-x_offset_lanelets, -y_offset_lanelets]), 0)

        traffic_signs = scenario_source.lanelet_network.traffic_signs

        for id_segment in range(num_segments):
            # generate scenario of current segment
            scenario_segment = generate_scenarios_without_problems(id_segment, scenario_duration, dt, lanelet_network,
                                                                   dict_tracks, traffic_signs)
            # skip if there is only a few obstacles in the scenario
            if len(scenario_segment.dynamic_obstacles) < 3:
                continue

            # we generate individual and cooperative planning problems separately
            num_scenarios_normal_indi = 2
            num_scenarios_survival_indi = 1
            num_scenarios_normal_coop = 1
            num_scenarios_survival_coop = 1

            id_config_scenario_indi = generate_scenario_with_individual_problems(id_segment, id_config_scenario_indi,
                                                                                 copy.copy(scenario_segment),
                                                                                 scenario_duration,
                                                                                 num_scenarios_normal_indi,
                                                                                 num_scenarios_survival_indi,
                                                                                 prefix_name, directory_output,
                                                                                 flag_same_direction_problems, tags,
                                                                                 obstacle_start_at_zero,
                                                                                 num_planning_problems,
                                                                                 keep_ego)
            id_config_scenario_coop = generate_scenarios_with_cooperative_problems(id_segment, id_config_scenario_coop,
                                                                                   copy.copy(scenario_segment),
                                                                                   scenario_duration,
                                                                                   num_scenarios_normal_coop,
                                                                                   num_scenarios_survival_coop,
                                                                                   prefix_name, directory_output,
                                                                                   flag_same_direction_problems, tags,
                                                                                   obstacle_start_at_zero,
                                                                                   num_planning_problems,
                                                                                   keep_ego)

            if (id_segment + 1) % 10 == 0 or (id_segment + 1) == num_segments: print(
                f"\t{id_segment + 1} / {num_segments} segments processed.")

    id_config_scenario_indi -= 1
    id_config_scenario_coop -= 1
    print(
        f"\tGenerated scenarios: individual: {id_config_scenario_indi}, cooperative: {id_config_scenario_coop}, total: {id_config_scenario_indi + id_config_scenario_coop}")
    return id_config_scenario_indi, id_config_scenario_coop
