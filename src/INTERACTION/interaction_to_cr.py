__author__ = "Edmond Irani Liu, Xiao Wang"
__copyright__ = "TUM Cyber-Physical Systems Group"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Release"

import os
import multiprocessing

from typing import Union
from src.INTERACTION.src.config import get_list_info_dataset
from src.INTERACTION.src.converter import generate_scenarios

from commonroad.scenario.scenario import Tag


def create_interaction_scenarios(input_dir: str, output_dir: str = "scenarios_converted/", directory_maps: Union[str, None] = None,
                                 obstacle_start_at_zero: bool = True, num_planning_problems: int = 1,
                                 keep_ego: bool = False,
                                 num_time_steps_scenario: int = 150,  num_processes: int = 1):
    if directory_maps is None:
        directory_maps = os.path.dirname(os.path.abspath(__file__)) + "/repaired_maps"
    """

    converts the scenarios in lanlet2 format into commonroad format


    :param directory_maps: path to folder with the .xml files of the maps
    """

    # get config info
    list_info_dataset = get_list_info_dataset(input_dir, directory_maps, output_dir)
    print(f"Number of maps to be processed: {len(list_info_dataset)}")

    # iterate through the config and process the scenarios
    sum_scenarios_indi = sum_scenarios_coop = 0
    if num_processes < 2:
        for idx, info in enumerate(list_info_dataset):
            print(f"\nProcessing {idx + 1} / {len(list_info_dataset)}:")

        num_scenarios_indi, num_scenario_coop = \
            generate_scenarios(prefix_name=info['prefix_name'],
                               path_map=info['path_map'],
                               directory_data=info['directory_data'],
                               directory_output=info['directory_output'],
                               flag_same_direction_problems=info.get('flag_same_direction_problems', False),
                               tags=[Tag(tag) for tag in info['tags'].split(' ')],
                               x_offset_lanelets=info['x_offset_lanelets'],
                               y_offset_lanelets=info['y_offset_lanelets'],
                               x_offset_tracks=info['x_offset_tracks'],
                               y_offset_tracks=info['y_offset_tracks'],
                               obstacle_start_at_zero=obstacle_start_at_zero,
                               num_planning_problems=num_planning_problems,
                               keep_ego=keep_ego,
                               scenario_duration=num_time_steps_scenario)

            sum_scenarios_indi += num_scenarios_indi
            sum_scenarios_coop += num_scenario_coop

        print(f"""\nGenerated scenarios: individual: {sum_scenarios_indi}, cooperative: {sum_scenarios_coop}, \
            total: {sum_scenarios_indi + sum_scenarios_coop}""")

        print(f"\nGenerated scenarios: individual: {sum_scenarios_indi}, cooperative: {sum_scenarios_coop}, \
            total: {sum_scenarios_indi + sum_scenarios_coop}")
    else:
        with multiprocessing.Pool(processes=num_processes) as pool:
            pool.starmap(
                generate_scenarios,
                [
                    (
                        info['prefix_name'],
                        info['path_map'],
                        info['directory_data'],
                        info['directory_output'],
                        info.get('flag_same_direction_problems', False),
                        [Tag(tag) for tag in info['tags'].split(' ')],
                        info['x_offset_lanelets'],
                        info['y_offset_lanelets'],
                        info['x_offset_tracks'],
                        info['y_offset_tracks'],
                        obstacle_start_at_zero=obstacle_start_at_zero,
                        num_planning_problems=num_planning_problems,
                        keep_ego=keep_ego,
                        scenario_duration=num_time_steps_scenario
                    ) for idx, info in enumerate(list_info_dataset)
                ]
            )
