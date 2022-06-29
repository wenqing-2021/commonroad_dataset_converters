__author__ = "Niels Mündler, Xiao Wang"
__copyright__ = "TUM Cyber-Physical System Group"
__credits__ = [""]
__version__ = "1.0"
__maintainer__ = "Xiao Wang"
__email__ = "xiao.wang@tum.de"
__status__ = "Released"

__desc__ = """
Converts inD files to Commonroad Format, creating smaller Planning Problems if required
"""

import os
import glob
import copy
import math
import random
import logging
import multiprocessing
import pandas as pd
from typing import Dict, Union, Type

from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.common.file_writer import CommonRoadFileWriter, Tag, OverwriteExistingFile

from commonroad_dataset_converter.helper import load_yaml
from commonroad_dataset_converter.inD.map_utils import load_lanelet_networks, meta_scenario_from_recording
from commonroad_dataset_converter.inD.obstacle_utils import generate_obstacle
from commonroad_dataset_converter.planning_problem_utils import generate_planning_problem, NoCarException, \
    obstacle_to_planning_problem, \
    check_routability_planning_problem, Routability

LOGGER = logging.getLogger(__name__)


def generate_single_scenario(ind_config: Dict, num_planning_problems: int, keep_ego: bool, output_dir: str,
                             tracks_df: pd.DataFrame, tracks_meta_df: pd.DataFrame, meta_scenario: Scenario,
                             benchmark_id: str, frame_start: int, frame_end: int,
                             obstacle_start_at_zero: bool, ego_vehicle_id: int = None,
                             routability_planning_problem: Type[Routability] = Routability.ANY):
    """
    Generate a single CommonRoad scenario based on inD record snippet
    :param ind_config: dictionary with configuration parameters for highD scenario generation
    :param num_planning_problems: number of planning problems per CommonRoad scenario
    :param output_dir: path to store generated CommonRoad scenario files
    :param keep_ego: boolean indicating if vehicles selected for planning problem should be kept in scenario
    :param tracks_df: single track
    :param tracks_meta_df: single meta information track
    :param meta_scenario: CommonRoad scenario with lanelet network
    :param benchmark_id: CommonRoad benchmark ID for scenario
    :param frame_start: start of frame in time steps of record
    :param frame_end: end of frame in time steps of record
    :param obstacle_start_at_zero: boolean indicating if the initial state of an obstacle has to start
    at time step zero
    :param ego_vehicle_id: None if random select ego vehicle from all converted cars
    """

    def enough_time_steps(veh_id: int):
        vehicle_meta = tracks_meta_df[tracks_meta_df.trackId == veh_id]
        if not obstacle_start_at_zero and frame_end - int(vehicle_meta.initialFrame) < 2 \
                or int(vehicle_meta.finalFrame) - frame_start < 2:
            return False
        elif obstacle_start_at_zero and int(vehicle_meta.initialFrame) > frame_start \
                or int(vehicle_meta.finalFrame) - frame_start < 2:
            return False
        return True

    # copy meta_scenario with lanelet networks
    scenario = copy.deepcopy(meta_scenario)
    scenario.scenario_id = benchmark_id

    # read tracks appear between [frame_start, frame_end]
    scenario_tracks_df = tracks_df[(tracks_df.frame >= frame_start) & (tracks_df.frame <= frame_end)]
    planning_problem_set = PlanningProblemSet()

    if ego_vehicle_id is not None:
        # create obstacle and planning problem from this track of ego car
        ego_obstacle = generate_obstacle(
            scenario_tracks_df,
            tracks_meta_df,
            vehicle_id=ego_vehicle_id,
            obstacle_id=scenario.generate_object_id(),
            frame_start=frame_start,
            class_to_type=ind_config.get("class_to_obstacleType"),
            detect_static_vehicles=False
        )
        if keep_ego:
            scenario.add_objects(ego_obstacle)
            planning_problem_id = scenario.generate_object_id()
        else:
            planning_problem_id = ego_obstacle.obstacle_id

        planning_problem = obstacle_to_planning_problem(obstacle=ego_obstacle,
                                                        planning_problem_id=planning_problem_id,
                                                        lanelet_network=scenario.lanelet_network)
        if routability_planning_problem and check_routability_planning_problem(
                scenario, planning_problem, max_difficulity=routability_planning_problem
        ):
            planning_problem_set.add_planning_problem(planning_problem)

        else:
            pass  # skip this planning problem, it is not possible.

        num_planning_problems -= 1

    # generate CR obstacles
    for vehicle_id in [vehicle_id for vehicle_id in scenario_tracks_df.trackId.unique()
                       if vehicle_id in tracks_meta_df.trackId.unique()]:
        # if appearing time steps < min_time_steps, skip vehicle
        if not enough_time_steps(vehicle_id):
            continue
        if ego_vehicle_id is not None and vehicle_id == ego_vehicle_id:
            continue
        print("Generating scenario {}, vehicle id {}".format(benchmark_id, vehicle_id), end="\r")
        obstacle = generate_obstacle(
            scenario_tracks_df,
            tracks_meta_df,
            vehicle_id=vehicle_id,
            obstacle_id=scenario.generate_object_id(),
            frame_start=frame_start,
            class_to_type=ind_config.get("class_to_obstacleType"),
            detect_static_vehicles=False
        )
        scenario.add_objects(obstacle)

    # generate planning problems

    duplicate_prevent_counter = -1

    for _ in range(num_planning_problems):
        for i in range(len(scenario.dynamic_obstacles)):

            # If num_planning_problems is greater than 1, ensure that an actual new planning_problem is
            # added by skipping all already used (and checked) dynamic obstacles
            if i <= duplicate_prevent_counter:
                continue
            try:
                planning_problem = generate_inD_planning_problem(scenario, keep_ego=keep_ego, count=i)
            except: 
                continue    
            if routability_planning_problem:
                if not check_routability_planning_problem(
                        scenario, planning_problem, max_difficulity=routability_planning_problem
                ):
                    continue  # skip this planning problem, it is not routeable.
            planning_problem_set.add_planning_problem(planning_problem)
            duplicate_prevent_counter = i
            break

    # check that there is at least one planning problem
    if len(planning_problem_set.planning_problem_dict.keys()) == 0:
        print(f"no planning problem possible for {scenario.scenario_id}")
        return

    # write new scenario
    tags = {Tag(tag) for tag in ind_config.get("tags")}
    fw = CommonRoadFileWriter(scenario, planning_problem_set, ind_config.get("author"),
                              ind_config.get("affiliation"), ind_config.get("source"), tags)
    filename = os.path.join(output_dir, "{}.xml".format(scenario.scenario_id))

    # Do not check validity if obstacles do not start at zero because validity will not pass
    fw.write_to_file(filename, OverwriteExistingFile.ALWAYS, check_validity=obstacle_start_at_zero)
    print(f"Scenario file stored in {filename}")
    return


def load_data(recording_meta_fn: str, tracks_meta_fn: str, tracks_fn: str, ind_config: Dict):
    # read data frames from the three files
    recording_meta_df = pd.read_csv(recording_meta_fn, header=0)
    tracks_meta_df = pd.read_csv(tracks_meta_fn, header=0)
    tracks_df = pd.read_csv(tracks_fn, header=0)

    # generate meta scenario with lanelet network
    meta_scenario = meta_scenario_from_recording(
        ind_config,
        recording_meta_df.locationId.values[0],
        recording_meta_df.recordingId.values[0],
        recording_meta_df.frameRate.values[0],
    )

    return recording_meta_df, tracks_meta_df, tracks_df, meta_scenario


def construct_benchmark_id(ind_config, recording_meta_df, idx_1):
    return "DEU_{0}-{1}_{2}_T-1".format(
        ind_config.get("location_benchmark_id")[recording_meta_df.locationId.values[0]],
        int(recording_meta_df.recordingId), idx_1 + 1)


def generate_scenarios_for_record(recording_meta_fn: str, tracks_meta_fn: str, tracks_fn: str,
                                  num_time_steps_scenario: int, num_planning_problems: int, keep_ego: bool,
                                  output_dir: str, ind_config: Dict, obstacle_start_at_zero: bool,
                                  routability_planning_problem: Type[Routability]):
    """
    Generate CommonRoad scenarios with given paths to highD for a high-D recording

    :param recording_meta_fn: path to *_recordingMeta.csv
    :param tracks_meta_fn: path to *_tracksMeta.csv
    :param tracks_fn: path to *_tracks.csv
    :param num_time_steps_scenario: maximal number of time steps per CommonRoad scenario
    :param num_planning_problems: number of planning problems per CommonRoad scenario
    :param keep_ego: boolean indicating if vehicles selected for planning problem should be kept in scenario
    :param output_dir: path to store generated CommonRoad scenario files
    :param ind_config: dictionary with configuration parameters for inD scenario generation
    :param obstacle_start_at_zero: boolean indicating if the initial state of an obstacle has to start
    at time step zero
    :param:  routability_planning_problem: type Routability, kind of routing check to perform on planning_problem
    """
    recording_meta_df, tracks_meta_df, tracks_df, meta_scenario = load_data(recording_meta_fn, tracks_meta_fn,
                                                                            tracks_fn, ind_config)

    # separate record and generate scenario for each separated part for each direction
    # (upper interstate direction / lower interstate direction)
    num_scenarios = math.ceil(max(tracks_meta_df.finalFrame) / num_time_steps_scenario)
    for idx_1 in range(num_scenarios):
        # benchmark id format: COUNTRY_SCENE_CONFIG_PRED
        frame_start = idx_1 * num_time_steps_scenario + (idx_1 + 1)
        frame_end = (idx_1 + 1) * num_time_steps_scenario + (idx_1 + 1)
        benchmark_id = construct_benchmark_id(ind_config, recording_meta_df, idx_1)
        try:
            generate_single_scenario(
                ind_config=ind_config, num_planning_problems=num_planning_problems, keep_ego=keep_ego,
                output_dir=output_dir,
                tracks_df=tracks_df, tracks_meta_df=tracks_meta_df, meta_scenario=meta_scenario,
                benchmark_id=benchmark_id, frame_start=frame_start, frame_end=frame_end,
                obstacle_start_at_zero=obstacle_start_at_zero, ego_vehicle_id=None,
                routability_planning_problem=routability_planning_problem
            )

        except NoCarException as e:
            print(f"No car in this scenario: {repr(e)}. Skipping this scenario.")


def generate_scenarios_for_record_vehicle(recording_meta_fn: str, tracks_meta_fn: str, tracks_fn: str,
                                          num_time_steps_scenario: int, num_planning_problems: int, keep_ego: bool,
                                          output_dir: str, ind_config: Dict, obstacle_start_at_zero: bool,
                                          routability_planning_problem: Type[Routability]):
    """
    Generate CommonRoad scenarios with given paths to inD for an inD recording

    :param recording_meta_fn: path to *_recordingMeta.csv
    :param tracks_meta_fn: path to *_tracksMeta.csv
    :param tracks_fn: path to *_tracks.csv
    :param num_time_steps_scenario: maximal number of time steps per CommonRoad scenario
    :param num_planning_problems: number of planning problems per CommonRoad scenario
    :param keep_ego: boolean indicating if vehicles selected for planning problem should be kept in scenario
    :param output_dir: path to store generated CommonRoad scenario files
    :param ind_config: dictionary with configuration parameters for inD scenario generation
    :param obstacle_start_at_zero: boolean indicating if the initial state of an obstacle has to start
    at time step zero
    :param  routability_planning_problem: type Routability, kind of routing check to perform on planning_problem
    """
    recording_meta_df, tracks_meta_df, tracks_df, meta_scenario = load_data(recording_meta_fn, tracks_meta_fn,
                                                                            tracks_fn, ind_config)

    # iterate all cars and create one scenario for each car
    tracks_meta_df_car = tracks_meta_df[(tracks_meta_df["class"] == "car") &
                                        (tracks_meta_df.numFrames >= num_time_steps_scenario)]
    time_step_half_range = 25

    for ego_vehicle_id in tracks_meta_df_car.trackId.unique():
        track_df_vehicle = tracks_df[tracks_df.trackId == ego_vehicle_id]
        max_velocity = max(track_df_vehicle.xVelocity ** 2 + track_df_vehicle.yVelocity ** 2)
        if max_velocity > 10.:
            # select this moving vehicle as ego vehicle
            # cut tracks_df into track_df_vehicle
            frame_start = min(track_df_vehicle.frame)
            frame_end = max(track_df_vehicle.frame) + time_step_half_range

            benchmark_id = construct_benchmark_id(ind_config, recording_meta_df, ego_vehicle_id)
            try:
                generate_single_scenario(
                    ind_config=ind_config, num_planning_problems=num_planning_problems, keep_ego=keep_ego,
                    output_dir=output_dir, tracks_df=tracks_df, tracks_meta_df=tracks_meta_df,
                    meta_scenario=meta_scenario,
                    benchmark_id=benchmark_id, frame_start=frame_start, frame_end=frame_end,
                    obstacle_start_at_zero=obstacle_start_at_zero, ego_vehicle_id=ego_vehicle_id,
                    routability_planning_problem=routability_planning_problem
                )
            except IndexError as e:
                print(f"Cannot find lanelet by position: {repr(e)}. Skipping this scenario.")


def create_ind_scenarios(input_dir: str, output_dir: str, num_time_steps_scenario: int,
                         num_planning_problems: int, keep_ego: bool, obstacle_start_at_zero: bool,
                         map_dir: Union[str, None] = None, seed: int = 0,
                         verbose: bool = True, num_processes: int = 1, inD_all: bool = False,
                         routability_planning_problem=Routability.ANY):
    if verbose:
        LOGGER.setLevel(logging.INFO)
        logging.basicConfig(level=logging.INFO)

    # Create output dir if necessary
    os.makedirs(output_dir, exist_ok=True)

    if map_dir is None:
        map_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "repaired_maps")

    # set the seed for random slices
    random.seed(seed)

    # generate path to highd data files
    path_tracks = os.path.join(input_dir, "data/*_tracks.csv")
    path_metas = os.path.join(input_dir, "data/*_tracksMeta.csv")
    path_recording = os.path.join(input_dir, "data/*_recordingMeta.csv")

    # get all file names
    listing_tracks = sorted(glob.glob(path_tracks))
    listing_metas = sorted(glob.glob(path_metas))
    listing_recording = sorted(glob.glob(path_recording))

    ind_config = load_yaml(os.path.dirname(os.path.abspath(__file__)) + "/config.yaml")
    load_lanelet_networks(map_dir, ind_config=ind_config)

    if inD_all:
        fn = generate_scenarios_for_record_vehicle
    else:
        fn = generate_scenarios_for_record

    if num_processes < 2:
        for index, (recording_meta_fn, tracks_meta_fn, tracks_fn) in \
                enumerate(zip(listing_recording, listing_metas, listing_tracks)):
            print("=" * 80)
            print("Processing file {}...".format(tracks_fn), end='\n')
            fn(recording_meta_fn, tracks_meta_fn, tracks_fn, num_time_steps_scenario,
               num_planning_problems, keep_ego, output_dir, ind_config,
               obstacle_start_at_zero, Routability(routability_planning_problem))
    else:
        with multiprocessing.Pool(processes=num_processes) as pool:
            pool.starmap(
                fn,
                [
                    (
                        recording_meta_fn,
                        tracks_meta_fn,
                        tracks_fn,
                        num_time_steps_scenario,
                        num_planning_problems,
                        keep_ego,
                        output_dir,
                        ind_config,
                        obstacle_start_at_zero,
                        Routability(routability_planning_problem)
                    )
                    for recording_meta_fn, tracks_meta_fn, tracks_fn in \
                    zip(listing_recording, listing_metas, listing_tracks)
                ]
            )


def generate_inD_planning_problem(scenario: Scenario, orientation_half_range: float = 0.2,
                                  velocity_half_range: float = 10.,
                                  time_step_half_range: int = 25, keep_ego: bool = False,
                                  lane_change: bool = False, count: int = 0) -> PlanningProblem:
    """
    Generates planning problem for scenario by taking obstacle trajectory
    :param scenario: CommonRoad scenario
    :param orientation_half_range: parameter for goal state orientation
    :param velocity_half_range: parameter for goal state velocity
    :param time_step_half_range: parameter for goal state time step
    :param keep_ego: boolean indicating if vehicles selected for planning problem should be kept in scenario
    :param count: indicator for which dynamic obstacle to use as ego vehicle
    :return: CommonRoad planning problem
    """

    if len(scenario.dynamic_obstacles) == 0:
        raise NoCarException("There is no car in dynamic obstacles which can be used as planning problem.")

    max_time_step = max([obstacle.prediction.final_time_step for obstacle in scenario.dynamic_obstacles])
    # only choose car type as ego vehicle
    car_obstacles = [obstacle for obstacle in scenario.dynamic_obstacles if
                     (obstacle.obstacle_type == ObstacleType.CAR
                      and obstacle.initial_state.time_step == 0
                      )]

    if len(car_obstacles) > 0:
        dynamic_obstacle_selected = scenario.dynamic_obstacles[count]
    else:
        raise NoCarException("There is no car in dynamic obstacles which can be used as planning problem.")

    if not keep_ego:
        planning_problem_id = dynamic_obstacle_selected.obstacle_id
        scenario.remove_obstacle(dynamic_obstacle_selected)
    else:
        planning_problem_id = scenario.generate_object_id()

    if len(scenario.dynamic_obstacles) > 0:
        max_time_step = max(
            [obs.prediction.trajectory.state_list[-1].time_step for obs in scenario.dynamic_obstacles])
        final_time_step = min(
            dynamic_obstacle_selected.prediction.trajectory.final_state.time_step + time_step_half_range,
            max_time_step)
    else:
        final_time_step = dynamic_obstacle_selected.prediction.trajectory.final_state.time_step + time_step_half_range

    planning_problem = obstacle_to_planning_problem(dynamic_obstacle_selected,
                                                    planning_problem_id,
                                                    final_time_step=final_time_step,
                                                    orientation_half_range=orientation_half_range,
                                                    velocity_half_range=velocity_half_range,
                                                    time_step_half_range=time_step_half_range,
                                                    lanelet_network=scenario.lanelet_network,
                                                    highD=False)

    return planning_problem
