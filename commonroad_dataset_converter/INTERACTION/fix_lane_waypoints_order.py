import os

import numpy as np

from argparse import ArgumentParser

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile, Tag
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad_dataset_converter.helper import load_yaml


def get_parser():
    parser = ArgumentParser()
    parser.add_argument("-i", default="./repaired_maps/translated", dest="input",
                        help="Path to directory containing commonroad formatted lanelets (converts all files)")
    parser.add_argument("-o", default="./repaired_maps/translated/fixed", help="Path to output directory",
                        dest="output")

    return parser.parse_args()


def is_waypoints_order_correct(lanelet: Lanelet, lanelet_network: LaneletNetwork):
    if lanelet.lanelet_id == 102:
        p = 1
    is_order_correct = len(lanelet.predecessor) == 0 and len(lanelet.successor) == 0
    if len(lanelet.predecessor) > 0:
        predecessor = lanelet_network.find_lanelet_by_id(lanelet.predecessor[0])
        is_order_correct = np.allclose(predecessor.left_vertices[-1], lanelet.left_vertices[0])
    if len(lanelet.successor) > 0:
        successor = lanelet_network.find_lanelet_by_id(lanelet.successor[0])
        is_order_correct = np.allclose(successor.left_vertices[0], lanelet.left_vertices[-1])

    return is_order_correct


def fix_waypoints_order(lanelet: Lanelet):
    return Lanelet(left_vertices=np.flip(lanelet.left_vertices), center_vertices=np.flip(lanelet.center_vertices),
            right_vertices=np.flip(lanelet.right_vertices), lanelet_id=lanelet.lanelet_id,
            predecessor=lanelet.predecessor, successor=lanelet.successor, adjacent_left=lanelet.adj_left,
            adjacent_left_same_direction=lanelet.adj_left_same_direction, adjacent_right=lanelet.adj_right,
            adjacent_right_same_direction=lanelet.adj_right_same_direction,
            line_marking_left_vertices=lanelet.line_marking_left_vertices,
            line_marking_right_vertices=lanelet.line_marking_right_vertices, stop_line=lanelet.stop_line,
            lanelet_type=lanelet.lanelet_type, user_one_way=lanelet.user_one_way,
            user_bidirectional=lanelet.user_bidirectional, traffic_signs=lanelet.traffic_signs,
            traffic_lights=lanelet.traffic_lights)


if __name__ == "__main__":
    args = get_parser()

    os.makedirs(args.output, exist_ok=True)

    interaction_config = load_yaml(os.path.dirname(os.path.abspath(__file__)) + "/config.yaml")

    author = interaction_config["author"]
    affiliation = interaction_config["affiliation"]
    source = interaction_config["source"]

    for location in interaction_config["locations"].values():
        if location == "DEU_Merging-1":
            file_path = os.path.join(args.input, f"{interaction_config['maps'][location]}.xml")
            scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
            # x_offset_lanelets = interaction_config["offsets"][location]["x_offset_lanelets"]
            # y_offset_lanelets = interaction_config["offsets"][location]["y_offset_lanelets"]
            tags = [Tag(tag) for tag in interaction_config['tags'][location].split(' ')]

            scenario_new = Scenario(dt=scenario.dt, scenario_id=scenario.scenario_id)

            for lanelet in scenario.lanelet_network.lanelets:
                # check if order of vertices is wrong
                if is_waypoints_order_correct(lanelet, scenario.lanelet_network):
                    scenario_new.add_objects(lanelet)
                    assert lanelet.lanelet_id in [100, 101, 109, 105]
                else:
                    assert lanelet.lanelet_id in [102, 106, 110, 103, 107, 111, 108, 104, 112]
                    scenario_new.add_objects(fix_waypoints_order(lanelet))

            scenario_new.add_objects(scenario.lanelet_network.intersections)
            scenario_new.add_objects(scenario.lanelet_network.traffic_lights)
            scenario_new.add_objects(scenario.lanelet_network.traffic_signs)

            file_writer = CommonRoadFileWriter(scenario, planning_problem_set, author, affiliation, source, tags)
            output_file = os.path.join(args.output, f"{interaction_config['maps'][location]}.xml")
            file_writer.write_to_file(output_file, OverwriteExistingFile.ALWAYS)
