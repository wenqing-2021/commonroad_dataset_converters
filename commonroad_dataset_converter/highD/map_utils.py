import numpy as np
from typing import Union, List
from pandas import DataFrame
from enum import Enum

from commonroad.scenario.scenario import Scenario, ScenarioID
from commonroad.scenario.lanelet import Lanelet, LaneletType, RoadUser, LineMarking
from commonroad.scenario.traffic_sign import TrafficSignIDGermany, TrafficSignElement, TrafficSign
from commonroad.geometry.polyline_util import resample_polyline_with_distance


class Direction(Enum):
    """
    Enum for representing upper or lower interstate road
    """
    UPPER = 1
    LOWER = 2


def get_lane_markings(recording_df: DataFrame, extend_width=0.):
    """
    Extracts upper and lower lane markings from data frame;

    :param recording_df: data frame of the recording meta information
    :param extend_width: extend width of the outer lanes,
                         set to a positive value when some vehicles are
                         off-road at the first time step.
    :return: speed limit
    """
    upper_lane_markings = [-float(x) for x in recording_df.upperLaneMarkings.values[0].split(";")]
    lower_lane_markings = [-float(x) for x in recording_df.lowerLaneMarkings.values[0].split(";")]
    len_upper = len(upper_lane_markings)
    len_lower = len(lower_lane_markings)
    # -8 + 1 = -7
    upper_lane_markings[0] += extend_width
    # -16 -1 = -17
    upper_lane_markings[len_upper - 1] += -extend_width
    # -22 + 1 = -21
    lower_lane_markings[0] += extend_width
    # -30 -1 = -31
    lower_lane_markings[len_lower - 1] += -extend_width
    return upper_lane_markings, lower_lane_markings


def get_dt(recording_df: DataFrame) -> float:
    """
    Extracts time step size from data frame

    :param recording_df: data frame of the recording meta information
    :return: time step size
    """
    return 1. / recording_df.frameRate.values[0]


def get_speed_limit(recording_df: DataFrame) -> Union[float, None]:
    """
    Extracts speed limit from data frame

    :param recording_df: data frame of the recording meta information
    :return: speed limit
    """
    speed_limit = recording_df.speedLimit.values[0]
    if speed_limit < 0:
        return None
    else:
        return speed_limit


def get_meta_scenario(dt: float, benchmark_id: str, lane_markings: List[float], speed_limit: float, road_length: int,
                      direction: Direction, road_offset: int, shoulder: bool, shoulder_width: float,
                      num_vertices: int = 10):
    """
    Generates meta CommonRoad scenario containing only lanelet network

    :param dt: time step size
    :param benchmark_id: benchmark ID of meta scenario
    :param lane_markings: list of y-positions for lane markings
    :param speed_limit: speed limits for road
    :param road_length: length of road
    :param direction: indicator for upper or lower interstate road
    :param road_offset: length added on both sides of road
    :param shoulder: indicates whether shoulder lane should be added to scenario
    :param shoulder_width: width of shoulder lane
    :param num_vertices: number of waypoints of lanes
    :return: CommonRoad scenario
    """
    scenario = Scenario(dt, ScenarioID.from_benchmark_id(benchmark_id, "2020a"))
    resample_step = (road_offset + 2 * road_offset) / num_vertices
    lanelet_id = 0
    if direction is Direction.UPPER:
        if shoulder:
            lane_y = lane_markings[0] + shoulder_width
            next_lane_y = lane_markings[0]

            right_vertices = resample_polyline_with_distance(
                    np.array([[road_length + road_offset, lane_y], [-road_offset, lane_y]]), distance=resample_step)
            left_vertices = resample_polyline_with_distance(
                    np.array([[road_length + road_offset, next_lane_y], [-road_offset, next_lane_y]]),
                    distance=resample_step)
            center_vertices = (left_vertices + right_vertices) / 2.0

            # assign lanelet ID and adjacent IDs and lanelet types
            lanelet_id = lanelet_id + 1
            lanelet_type = {LaneletType.INTERSTATE, LaneletType.SHOULDER}
            adjacent_left = lanelet_id + 1
            adjacent_right = None
            adjacent_left_same_direction = True
            adjacent_right_same_direction = False
            line_marking_left_vertices = LineMarking.SOLID
            line_marking_right_vertices = LineMarking.SOLID

            # add lanelet to scenario
            scenario.add_objects(
                    Lanelet(lanelet_id=lanelet_id, left_vertices=left_vertices, right_vertices=right_vertices,
                            center_vertices=center_vertices, adjacent_left=adjacent_left,
                            adjacent_left_same_direction=adjacent_left_same_direction, adjacent_right=adjacent_right,
                            adjacent_right_same_direction=adjacent_right_same_direction,
                            user_one_way={RoadUser.VEHICLE}, line_marking_left_vertices=line_marking_left_vertices,
                            line_marking_right_vertices=line_marking_right_vertices, lanelet_type=lanelet_type))
        for i in range(len(lane_markings) - 1):
            # get two lines of current lane
            lane_y = lane_markings[i]
            next_lane_y = lane_markings[i + 1]

            right_vertices = resample_polyline_with_distance(
                    np.array([[road_length + road_offset, lane_y], [-road_offset, lane_y]]), distance=resample_step)
            left_vertices = resample_polyline_with_distance(
                    np.array([[road_length + road_offset, next_lane_y], [-road_offset, next_lane_y]]),
                    distance=resample_step)
            center_vertices = (left_vertices + right_vertices) / 2.0

            # assign lanelet ID and adjacent IDs and lanelet types
            lanelet_id = lanelet_id + 1
            lanelet_type = {LaneletType.INTERSTATE, LaneletType.MAIN_CARRIAGE_WAY}
            adjacent_left = lanelet_id + 1
            adjacent_right = lanelet_id - 1
            adjacent_left_same_direction = True
            adjacent_right_same_direction = True
            line_marking_left_vertices = LineMarking.DASHED
            line_marking_right_vertices = LineMarking.DASHED

            if i == len(lane_markings) - 2:
                adjacent_left = None
                adjacent_left_same_direction = False
                line_marking_left_vertices = LineMarking.SOLID
            elif i == 0:
                if not shoulder:
                    adjacent_right = None
                    adjacent_right_same_direction = False
                line_marking_right_vertices = LineMarking.SOLID

            # add lanelet to scenario
            scenario.add_objects(
                    Lanelet(lanelet_id=lanelet_id, left_vertices=left_vertices, right_vertices=right_vertices,
                            center_vertices=center_vertices, adjacent_left=adjacent_left,
                            adjacent_left_same_direction=adjacent_left_same_direction, adjacent_right=adjacent_right,
                            adjacent_right_same_direction=adjacent_right_same_direction,
                            user_one_way={RoadUser.VEHICLE}, line_marking_left_vertices=line_marking_left_vertices,
                            line_marking_right_vertices=line_marking_right_vertices, lanelet_type=lanelet_type))
    else:
        for i in range(len(lane_markings) - 1):

            # get two lines of current lane
            next_lane_y = lane_markings[i + 1]
            lane_y = lane_markings[i]

            # get vertices of three lines
            # setting length 450 and -50 can cover all vehicle in this range
            left_vertices = resample_polyline_with_distance(
                    np.array([[-road_offset, lane_y], [road_length + road_offset, lane_y]]), distance=resample_step)
            right_vertices = resample_polyline_with_distance(
                    np.array([[-road_offset, next_lane_y], [road_length + road_offset, next_lane_y]]),
                    distance=resample_step)
            center_vertices = (left_vertices + right_vertices) / 2.0

            # assign lane ids and adjacent ids
            lanelet_id = lanelet_id + 1
            lanelet_type = {LaneletType.INTERSTATE, LaneletType.MAIN_CARRIAGE_WAY}
            adjacent_left = lanelet_id - 1
            adjacent_right = lanelet_id + 1
            adjacent_left_same_direction = True
            adjacent_right_same_direction = True
            line_marking_left_vertices = LineMarking.DASHED
            line_marking_right_vertices = LineMarking.DASHED

            if i == 0:
                adjacent_left = None
                adjacent_left_same_direction = False
                line_marking_left_vertices = LineMarking.SOLID
            elif i == len(lane_markings) - 2:
                if not shoulder:
                    adjacent_right = None
                    adjacent_right_same_direction = False
                line_marking_right_vertices = LineMarking.SOLID

            # add lanelet to scenario
            scenario.add_objects(
                    Lanelet(lanelet_id=lanelet_id, left_vertices=left_vertices, right_vertices=right_vertices,
                            center_vertices=center_vertices, adjacent_left=adjacent_left,
                            adjacent_left_same_direction=adjacent_left_same_direction, adjacent_right=adjacent_right,
                            adjacent_right_same_direction=adjacent_right_same_direction,
                            user_one_way={RoadUser.VEHICLE}, line_marking_left_vertices=line_marking_left_vertices,
                            line_marking_right_vertices=line_marking_right_vertices, lanelet_type=lanelet_type))
        if shoulder:
            # get two lines of current lane
            next_lane_y = lane_markings[-1] - shoulder_width
            lane_y = lane_markings[-1]

            # get vertices of three lines
            # setting length 450 and -50 can cover all vehicle in this range
            left_vertices = resample_polyline_with_distance(
                    np.array([[-road_offset, lane_y], [road_length + road_offset, lane_y]]), distance=resample_step)
            right_vertices = resample_polyline_with_distance(
                    np.array([[-road_offset, next_lane_y], [road_length + road_offset, next_lane_y]]),
                    distance=resample_step)
            center_vertices = (left_vertices + right_vertices) / 2.0

            # assign lane ids and adjacent ids
            lanelet_id = lanelet_id + 1
            lanelet_type = {LaneletType.INTERSTATE, LaneletType.SHOULDER}
            adjacent_left = lanelet_id - 1
            adjacent_right = None
            adjacent_left_same_direction = False
            adjacent_right_same_direction = True
            line_marking_left_vertices = LineMarking.SOLID
            line_marking_right_vertices = LineMarking.SOLID

            # add lanelet to scenario
            scenario.add_objects(
                    Lanelet(lanelet_id=lanelet_id, left_vertices=left_vertices, right_vertices=right_vertices,
                            center_vertices=center_vertices, adjacent_left=adjacent_left,
                            adjacent_left_same_direction=adjacent_left_same_direction, adjacent_right=adjacent_right,
                            adjacent_right_same_direction=adjacent_right_same_direction,
                            user_one_way={RoadUser.VEHICLE}, line_marking_left_vertices=line_marking_left_vertices,
                            line_marking_right_vertices=line_marking_right_vertices, lanelet_type=lanelet_type))
    # store speed limit for traffic sign generation
    if speed_limit is not None:
        traffic_sign_element = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED, [str(speed_limit)])
        position = scenario.lanelet_network.lanelets[0].right_vertices[0]
        lanelets = {lanelet.lanelet_id for lanelet in scenario.lanelet_network.lanelets}
        traffic_sign = TrafficSign(scenario.generate_object_id(), [traffic_sign_element], lanelets, position)

        scenario.add_objects(traffic_sign, lanelets)

    return scenario
