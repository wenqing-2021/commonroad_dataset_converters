__author__ = "Edmond Irani Liu"
__copyright__ = "TUM Cyber-Physical Systems Group"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Release"

#TODO remove dot again?
from src.INTERACTION.src.config import get_list_info_dataset
from src.INTERACTION.src.converter import generate_scenarios

from commonroad.scenario.scenario import Tag


def convert(directory_maps = "./maps_lanelet/",directory_output_scenrios = "scenarios_converted/",
            obstacle_initial_state_invalid:bool = True, num_planning_problems:int = 1, keep_ego:bool = False,
            num_time_steps_scenario:int =150):
    """

    converts the scenarios in lanlet2 format into commonroad format


    :param directory_maps: path to folder with the .xml files of the maps
    """

    # get config info
    list_info_dataset = get_list_info_dataset(directory_maps,directory_output_scenrios)
    print(f"Number of maps to be processed: {len(list_info_dataset)}")

    # iterate through the config and process the scenarios
    sum_scenarios_indi = sum_scenarios_coop = 0
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
                               obstacle_initial_state_invalid=obstacle_initial_state_invalid,
                               num_planning_problems=num_planning_problems,
                               keep_ego=keep_ego,
                               scenario_duration=num_time_steps_scenario)

        sum_scenarios_indi += num_scenarios_indi
        sum_scenarios_coop += num_scenario_coop

    print(f"""\nGenerated scenarios: individual: {sum_scenarios_indi}, cooperative: {sum_scenarios_coop}, \
        total: {sum_scenarios_indi + sum_scenarios_coop}""")

    print(f"\nGenerated scenarios: individual: {sum_scenarios_indi}, cooperative: {sum_scenarios_coop}, \
        total: {sum_scenarios_indi + sum_scenarios_coop}")