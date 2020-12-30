__author__ = "Edmond Irani Liu, Xiao Wang"
__copyright__ = "TUM Cyber-Physical Systems Group"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Release"

import os
import multiprocessing

from typing import Union
from src.INTERACTION.map_utils import generate_scenarios
from src.helper import load_yaml

from commonroad.scenario.scenario import Tag


def create_interaction_scenarios(input_dir: str, output_dir: str = "scenarios_converted/",
                                 directory_maps: Union[str, None] = None,
                                 obstacle_start_at_zero: bool = True, num_planning_problems: int = 1,
                                 keep_ego: bool = False,
                                 num_time_steps_scenario: int = 150, num_processes: int = 1):
    if directory_maps is None:
        directory_maps = os.path.dirname(os.path.abspath(__file__)) + "/repaired_maps"
    """

    converts the scenarios in lanelet2 format into commonroad format


    :param directory_maps: path to folder with the .xml files of the maps
    """

    # get config info
    assert os.path.exists(input_dir), "<dataset> folder not found under current working directory!"
    assert os.path.exists(directory_maps), "<maps_lanelet> folder not found under current working directory!"
    interaction_config = load_yaml(os.path.dirname(os.path.abspath(__file__)) + "/config.yaml")
    print(f"Number of maps to be processed: {len(list_info_dataset)}")

    # iterate through the config and process the scenarios
    sum_scenarios_indi = 0
    if num_processes < 2:
        for idx, location in enumerate(interaction_config['locations'].values()):

            offsets = interaction_config['offsets'][location]
            print(f"\nProcessing {idx + 1} / {len(interaction_config['locations'])}:")
            num_scenarios_indi = generate_scenarios(prefix_name=location + '_',
                                                    path_map=os.path.join(os.getcwd(), directory_maps,
                                                                          interaction_config['maps'][location])+'.xml',
                                                    directory_data=os.path.join(input_dir,
                                                                                interaction_config['directory_data'][
                                                                                    location]),
                                                    directory_output=os.path.join(os.getcwd(), output_dir,
                                                                                  f"{location}/"),
                                                    flag_same_direction_problems=interaction_config[
                                                        'flag_same_direction_problems'].get(
                                                        location, False),
                                                    tags=[Tag(tag) for tag in
                                                          interaction_config['tags'][location].split(' ')],
                                                    x_offset_lanelets=offsets['x_offset_lanelets'],
                                                    y_offset_lanelets=offsets['y_offset_lanelets'],
                                                    x_offset_tracks=offsets['x_offset_tracks'],
                                                    y_offset_tracks=offsets['y_offset_tracks'],
                                                    obstacle_start_at_zero=obstacle_start_at_zero,
                                                    num_planning_problems=num_planning_problems,
                                                    keep_ego=keep_ego,
                                                    scenario_time_steps=num_time_steps_scenario)
        sum_scenarios_indi += num_scenarios_indi

        print(f"""\nGenerated scenarios: {sum_scenarios_indi}""")

    else:
        with multiprocessing.Pool(processes=num_processes) as pool:
            pool.starmap(
                generate_scenarios,
                [
                    (
                        location + '_',
                        os.path.join(os.getcwd(), directory_maps, interaction_config['maps'][location]) + '.xml',
                        os.path.join(input_dir, interaction_config['directory_data'][location]),
                        os.path.join(os.getcwd(), output_dir, f"{location}/"),
                        interaction_config['flag_same_direction_problems'].get(location, False),
                        [Tag(tag) for tag in interaction_config['tags'][location].split(' ')],
                        interaction_config['offsets'][location]['x_offset_lanelets'],
                        interaction_config['offsets'][location]['y_offset_lanelets'],
                        interaction_config['offsets'][location]['x_offset_tracks'],
                        interaction_config['offsets'][location]['y_offset_tracks'],
                        num_time_steps_scenario,
                        obstacle_start_at_zero,
                        num_planning_problems,
                        keep_ego
                    ) for idx, location in enumerate(interaction_config['locations'].values())
                ]
            )
