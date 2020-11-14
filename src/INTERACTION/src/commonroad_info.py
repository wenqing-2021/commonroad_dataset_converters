#! /usr/bin/env python

__author__ = "Matthias Hamacher"
__copyright__ = ""
__credits__ = [""]
__version__ = "1.0"
__maintainer__ = "Xiao Wang"
__email__ = "xiao.wang@tum.de"
__status__ = ""

__desc__ = """
extracts information needed for conversion from xml files
"""

from argparse import ArgumentParser
from pathlib import Path
import numpy as np


from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import (
    CommonRoadFileWriter,
    OverwriteExistingFile,
    Tag,
)

from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.obstacle import DynamicObstacle, StaticObstacle
from commonroad.planning.planning_problem import PlanningProblem

from commonroad_route_planner.route_planner import RoutePlanner

from lxml import etree
from shapely.validation import explain_validity
from sys import exit

from typing import Tuple


def get_min(scenario: Scenario) -> Tuple[float, float]:
    '''
    finds the minimal x and y values of the given scenario
    :param scenario: commonroad Scenario
    :return: x minimum, y minimum
    '''
    min_x, min_y = 999999, 999999
    for l in scenario.lanelet_network.lanelets:
        l_min_x = min(np.min(l.left_vertices[:, 0]), np.min(l.right_vertices[:, 0]))
        l_min_y = min(np.min(l.left_vertices[:, 1]), np.min(l.right_vertices[:, 1]))
        if l_min_x < min_x:
            min_x = l_min_x
        if l_min_y < min_y:
            min_y = l_min_y

    return min_x, min_y

def get_max(scenario: Scenario) -> Tuple[float, float]:
    '''
    finds the minimal x and y values of the given scenario
    :param scenario: commonroad Scenario
    :return: x minimum, y minimum
    '''
    max_x, max_y = -999999, -9999999
    for l in scenario.lanelet_network.lanelets:
        l_max_x = max(np.max(l.left_vertices[:, 0]), np.max(l.right_vertices[:, 0]))
        l_max_y = max(np.min(l.left_vertices[:, 1]), np.max(l.right_vertices[:, 1]))
        if l_max_x > max_x:
            max_x = l_max_x
        if l_max_y > max_y:
            max_y = l_max_y

    return max_x, max_y

def calibrate_origin(scenario: Scenario) -> Scenario:
    '''
    calculates new origin and transforms scenario
    :param scenario: Commonroad scenario
    :return:
    '''
    min_x, min_y = get_min(scenario)
    # max_x, max_y = get_max(scenario)

    scenario.translate_rotate(-np.array([min_x, min_y]), 0)

    return scenario


def get_parser():
    parser = ArgumentParser(description=__desc__)
    parser.add_argument(
        "-i",
        help="Path to input file(s) (.xml commonroad scenario)",
        default="../dataset-converters/src/INTERACTION/maps_lanelet/CHN_Merging_ZS_repaired.xml",
        dest='input'
    )
    # parser.add_argument(
    #     "-s",
    #     help="Path to schema file (.xsd commonroad specification)",
    #     default="./XML_commonRoad_XSD_2020a.xsd",
    #     dest="spec",
    # )
    return parser

def main():
    args = get_parser().parse_args()
    xml_path = Path(args.input)
    scenario, planning_problem_set = CommonRoadFileReader(str(xml_path)).open()
    # min_x, min_y = get_max(scenario)
    # print(f'[{min_x}, {min_y}]')

    scenario = calibrate_origin(scenario)

    author = (
        "Matthias Hamacher"
    )
    # affiliation = ("Kalsruhe Institute of Technology, Germany", "ParisTech, France", "University of California, Berkeley")
    affiliation = "Technical University of Munich, Germany"
    source = "http://interaction-dataset.com//"
    tags = {Tag.INTERSECTION, Tag.MULTI_LANE, Tag.SPEED_LIMIT, Tag.URBAN} #scenario.tags #

    fw = CommonRoadFileWriter(scenario, planning_problem_set, author, affiliation, source, tags)
    fw.write_to_file(
        str(args.input), 
        OverwriteExistingFile.ALWAYS,
    )


# path = '../dataset-converters/src/INTERACTION/maps_lanelet/CHN_Merging_ZS_repaired.xml'

if __name__ == '__main__':
    main()

#commonroad_transform_check ../dataset-converters/src/INTERACTION/dataset/INTERACTION-Dataset-DR-v1_0/maps/DR_CHN_Merging_ZS.osm ../dataset-converters/src/INTERACTION/maps_lanelet/CHN_Merging_ZS_repaired.xml
