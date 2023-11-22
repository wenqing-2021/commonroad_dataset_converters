#! /usr/bin/env python

__author__ = "Niels Mündler, Xiao Wang"
__copyright__ = ""
__credits__ = [""]
__version__ = "1.0"
__maintainer__ = "Xiao Wang"
__email__ = "xiao.wang@tum.de"
__status__ = "Released"

__desc__ = """
Validates a Commonroad scenario syntactically
"""
import glob
import os
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import DynamicObstacle, StaticObstacle
from commonroad.scenario.scenario import Scenario
from lxml import etree
from shapely.validation import explain_validity


def check_adjacency(lanelet_network: LaneletNetwork):
    """
    Check that adjacencies are assigned correctly and bilaterally
    """
    errors = 0
    for lanelet in lanelet_network.lanelets:
        if lanelet.adj_left is not None:
            l_left = lanelet_network.find_lanelet_by_id(lanelet.adj_left)
            if (
                lanelet.adj_left_same_direction
                and l_left.adj_right != lanelet.lanelet_id
            ):
                print(
                    f"Left of lanelet {lanelet.lanelet_id} is {lanelet.adj_left} facing the same direction "
                    f"but right of {l_left.lanelet_id} is {l_left.adj_right}"
                )
                errors += 1
            if (
                not lanelet.adj_left_same_direction
                and l_left.adj_left != lanelet.lanelet_id
            ):
                print(
                    f"Left of lanelet {lanelet.lanelet_id} is {lanelet.adj_left} facing the opposite direction "
                    f"but left of {l_left.lanelet_id} is {l_left.adj_left}"
                )
                errors += 1
        if lanelet.adj_right is not None:
            l_right = lanelet_network.find_lanelet_by_id(lanelet.adj_right)
            if (
                lanelet.adj_right_same_direction
                and l_right.adj_left != lanelet.lanelet_id
            ):
                print(
                    f"Right of lanelet {lanelet.lanelet_id} is {lanelet.adj_right} facing the same direction "
                    f"but left of {l_right.lanelet_id} is {l_right.adj_left}"
                )
                errors += 1
            if (
                not lanelet.adj_right_same_direction
                and l_right.adj_right != lanelet.lanelet_id
            ):
                print(
                    f"Left of lanelet {lanelet.lanelet_id} is {lanelet.adj_right} facing the opposite direction "
                    f"but left of {l_right.lanelet_id} is {l_right.adj_left}"
                )
                errors += 1

    return errors


def check_valid_lanelet_polygon(lanelet_network: LaneletNetwork):
    errors = 0
    for lanelet in lanelet_network.lanelets:
        try:
            assert lanelet.polygon.shapely_object.is_valid
        except AssertionError as e:
            print(f"Lanelet {lanelet.lanelet_id} has invalid geometry")
            print(explain_validity(lanelet.polygon.shapely_object))
            errors += 1
    return errors


def check_successors(lanelet_network: LaneletNetwork):
    """
    Check that adjacencies are assigned correctly and bilaterally
    """
    errors = 0
    for lanelet in lanelet_network.lanelets:
        if lanelet.successor is not None:
            for i in lanelet.successor:
                suc = lanelet_network.find_lanelet_by_id(i)
                if not suc.predecessor or not lanelet.lanelet_id in suc.predecessor:
                    print(
                        f"Lanelet {i} is successor of {lanelet.lanelet_id} "
                        f"but does not have {lanelet.lanelet_id} as predecessor"
                    )
                    errors += 1
        if lanelet.predecessor is not None:
            for i in lanelet.predecessor:
                pred = lanelet_network.find_lanelet_by_id(i)
                if not pred.successor or not lanelet.lanelet_id in pred.successor:
                    print(
                        f"Lanelet {i} is predecessor of {lanelet.lanelet_id} "
                        f"but does not have {lanelet.lanelet_id} as successor"
                    )
                    errors += 1

    return errors


def check_obstacle_off_road(scenario: Scenario):
    errors = 0
    for obs in scenario.obstacles:
        off_road = True
        if isinstance(obs, DynamicObstacle):
            for s in obs.prediction.trajectory.state_list:
                if scenario.lanelet_network.find_lanelet_by_position([s.position]):
                    off_road = False
                    break
            else:
                off_road = True
        elif isinstance(obs, StaticObstacle):
            off_road = not scenario.lanelet_network.find_lanelet_by_position(
                [obs.initial_state.position]
            )

        if off_road:
            print(f"Obstacle {obs.obstacle_id} is off the road at all times")
            errors += 1

    return errors


def validate(xml_path: Path) -> int:
    # test whether a scenario file passes the schema test
    xsd_path = Path(__file__).parent / "XML_commonRoad_XSD_2020a.xsd"
    try:
        # assert xml_path.exists()
        etree.clear_error_log()
        xmlschema = etree.XMLSchema(etree.parse(str(xsd_path)))
        tmp = etree.parse(str(xml_path))
        xmlschema.assertValid(tmp)
    except AssertionError:
        print("File not found")
        return 1
    except etree.DocumentInvalid as e:
        print("File invalid: {}".format(e))
        return 1
    except Exception as e:
        print("Unknown error: {}".format(e))
        return 1
    return 0
    # assert xml_path.exists()
    # with xml_path.open("rb") as fp:
    #     xml_bytes = fp.read()
    # return CommonRoadFileWriter.check_validity_of_commonroad_file(xml_bytes)


def validate_all(path: Path, xsd_path: str):
    path = Path(path)
    xsd_path = Path(xsd_path)
    assert xsd_path.exists(), f"file path {xsd_path} doesn't exist"

    total_errors = 0

    fns = sorted(path.glob("*.xml"))
    for input_file in fns:
        print(f"Validating {input_file}")
        errors = 0

        # Check XML validity
        errors += validate(input_file)

        # Additionally check if adjacent right/left is always assigned respectively
        scenario, planning_problem_set = CommonRoadFileReader(input_file).open()
        lanelet_network = scenario.lanelet_network
        errors += check_adjacency(lanelet_network)

        # check for invalid lanelet polygons like self-intersections or similar
        errors += check_valid_lanelet_polygon(scenario.lanelet_network)

        # Check that no obstacle is off the road all the time
        errors += check_obstacle_off_road(scenario)

        total_errors += errors

    if total_errors > 0:
        print(f"{total_errors} Errors where detected")
        raise ValueError
    else:
        print("No errors")
