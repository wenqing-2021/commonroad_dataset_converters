__author__ = "Edmond Irani Liu"
__copyright__ = "TUM Cyber-Physical Systems Group"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Release"

from src.config import get_list_info_dataset
from src.converter import generate_scenarios


if __name__ == "__main__":
    # get config info
    list_info_dataset = get_list_info_dataset()
    print(f"Number of maps to be processed: {len(list_info_dataset)}")
    
    # iterate through the config and process the scenarios
    sum_scenarios_indi = sum_scenarios_coop = 0
    for idx, info in enumerate(list_info_dataset):
        print(f"\nProcessing {idx + 1} / {len(list_info_dataset)}:")

        num_scenarios_indi, num_scenario_coop = \
        generate_scenarios(prefix_name = info['prefix_name'], 
                           path_map = info['path_map'],
                           directory_data = info['directory_data'],
                           directory_output = info['directory_output'],
                           flag_same_direction_problems = info.get('flag_same_direction_problems', False),
                           tags = info['tags'], 
                           x_offset_lanelets = info['x_offset_lanelets'], 
                           y_offset_lanelets = info['y_offset_lanelets'],
                           x_offset_tracks = info['x_offset_tracks'], 
                           y_offset_tracks = info['y_offset_tracks'])

        sum_scenarios_indi += num_scenarios_indi
        sum_scenarios_coop += num_scenario_coop

    print(f"""\nGenerated scenarios: individual: {sum_scenarios_indi}, cooperative: {sum_scenarios_coop}, \
        total: {sum_scenarios_indi + sum_scenarios_coop}""")

    print(f"\nGenerated scenarios: individual: {sum_scenarios_indi}, cooperative: {sum_scenarios_coop}, \
        total: {sum_scenarios_indi + sum_scenarios_coop}")