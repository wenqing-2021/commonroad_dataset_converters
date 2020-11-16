__author__ = "Niels Mündler"
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
import pandas as pd
import multiprocessing
from typing import Dict, Union

from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.common.file_writer import CommonRoadFileWriter, Tag, OverwriteExistingFile

from src.helper import load_yaml
from src.inD.map_utils import load_lanelet_networks, meta_scenario_from_recording
from src.inD.obstacle_utils import generate_obstacle
from src.planning_problem_utils import generate_planning_problem, NoCarException

LOGGER = logging.getLogger(__name__)


def generate_single_scenario(ind_config: Dict, num_planning_problems: int, keep_ego: bool, output_dir: str,
                             tracks_df: pd.DataFrame, tracks_meta_df: pd.DataFrame, meta_scenario: Scenario,
                             benchmark_id: str, frame_start: int, frame_end: int,
                             obstacle_start_at_zero: bool):
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
    :param direction: indicator for upper or lower road of interstate
    :param frame_start: start of frame in time steps of record
    :param frame_end: end of frame in time steps of record
    :param obstacle_start_at_zero: boolean indicating if the initial state of an obstacle has to start
    at time step zero
    """

    def enough_time_steps(veh_id):
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
    scenario.benchmark_id = benchmark_id

    # read tracks appear between [frame_start, frame_end]
    scenario_tracks_df = tracks_df[(tracks_df.frame >= frame_start) & (tracks_df.frame <= frame_end)]

    # generate CR obstacles
    for vehicle_id in [vehicle_id for vehicle_id in scenario_tracks_df.trackId.unique()
                       if vehicle_id in tracks_meta_df.trackId.unique()]:
        # if appearing time steps < min_time_steps, skip vehicle
        if not enough_time_steps(vehicle_id):
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

    # return if scenario contains no dynamic obstacle
    if len(scenario.dynamic_obstacles) == 0:
        return

    # generate planning problems
    planning_problem_set = PlanningProblemSet()
    for idx_2 in range(num_planning_problems):
        planning_problem = generate_planning_problem(scenario, keep_ego=keep_ego)
        planning_problem_set.add_planning_problem(planning_problem)

    # write new scenario
    tags = {Tag(tag) for tag in ind_config.get("tags")}
    fw = CommonRoadFileWriter(scenario, planning_problem_set, ind_config.get("author"),
                              ind_config.get("affiliation"), ind_config.get("source"), tags)
    filename = os.path.join(output_dir, "{}.xml".format(scenario.benchmark_id))
    if obstacle_start_at_zero is True:
        check_validity = True
    else:
        # Do not check validity if obstacles do not start at zero because validity will not pass
        check_validity = False
    fw.write_to_file(filename, OverwriteExistingFile.ALWAYS, check_validity=check_validity)
    print("Scenario file stored in {}".format(filename))


def generate_scenarios_for_record(recording_meta_fn: str, tracks_meta_fn: str, tracks_fn: str,
                                  num_time_steps_scenario: int, num_planning_problems: int, keep_ego: bool,
                                  output_dir: str, ind_config: Dict, obstacle_start_at_zero: bool):
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
    """
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

    # separate record and generate scenario for each separated part for each direction
    # (upper interstate direction / lower interstate direction)
    num_scenarios = math.ceil(max(tracks_meta_df.finalFrame) / num_time_steps_scenario)
    for idx_1 in range(num_scenarios):
        # benchmark id format: COUNTRY_SCENE_CONFIG_PRED
        frame_start = idx_1 * num_time_steps_scenario + (idx_1 + 1)
        frame_end = (idx_1 + 1) * num_time_steps_scenario + (idx_1 + 1)
        benchmark_id = "DEU_{0}-{1}_{2}_T-1".format(
            ind_config.get("location_benchmark_id")[recording_meta_df.locationId.values[0]],
            int(recording_meta_df.recordingId), idx_1 + 1)
        try:
            generate_single_scenario(ind_config, num_planning_problems, keep_ego, output_dir, tracks_df,
                                     tracks_meta_df, meta_scenario, benchmark_id, frame_start,
                                     frame_end, obstacle_start_at_zero)
        except NoCarException as e:
            print(f"No car in this scenario: {repr(e)}. Skipping this scenario.")


def create_ind_scenarios(input_dir: str, output_dir: str, num_time_steps_scenario: int,
                         num_planning_problems: int, keep_ego: bool, obstacle_start_at_zero: bool,
                         map_dir: Union[str, None] = None, seed: int = 0,
                         verbose: bool = True, num_processes: int = 1):

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

    if num_processes < 2:
        for index, (recording_meta_fn, tracks_meta_fn, tracks_fn) in \
                enumerate(zip(listing_recording, listing_metas, listing_tracks)):
            print("=" * 80)
            print("Processing file {}...".format(tracks_fn), end='\n')
            generate_scenarios_for_record(recording_meta_fn, tracks_meta_fn, tracks_fn, num_time_steps_scenario,
                                          num_planning_problems, keep_ego, output_dir, ind_config,
                                          obstacle_start_at_zero)
    else:
        with multiprocessing.Pool(processes=num_processes) as pool:
            pool.starmap(
                generate_scenarios_for_record,
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
                        obstacle_start_at_zero
                    )
                    for recording_meta_fn, tracks_meta_fn, tracks_fn in \
                    zip(listing_recording, listing_metas, listing_tracks)
                ]
            )
