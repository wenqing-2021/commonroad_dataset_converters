__author__ = "Niels Mündler, Xiao Wang, Michael Feil"
__copyright__ = "TUM Cyber-Physical System Group"
__credits__ = [""]
__version__ = "1.0"
__maintainer__ = "Xiao Wang"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"

__desc__ = """
    Utilities for creating planning problems in the Converters
"""

from enum import Enum
from typing import Optional

import numpy as np
from commonroad.common.util import AngleInterval, Interval
from commonroad.geometry.shape import Polygon, Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.trajectory import State
from shapely.geometry import LineString, Point


class NoCarException(Exception):
    pass


class Routability(Enum):
    ANY = 0
    # REGULAR_ANDREVERSED = 1
    REGULAR_STRICT = 2


def _sub_line(start: Point, end: Point, line: LineString) -> LineString:
    """Get a section of a line by start and end point."""
    start_s = line.project(start)
    end_s = line.project(end)
    start_pt = line.interpolate(start_s)
    end_pt = line.interpolate(end_s)
    start_i = None
    for i, pt in enumerate(line.coords):
        s_pt = line.project(Point(pt))
        if s_pt > start_s and start_i is None:
            start_i = i
    end_i = None
    for i, pt in enumerate(reversed(line.coords)):
        s_pt = line.project(Point(pt))
        if s_pt > end_s and end_i is None:
            end_i = i
    pts = [start_pt] + line.coords[start_i:end_i] + [end_pt]
    return LineString(pts)


def _cut_lanelet_polygon(
    position: np.ndarray, lon_length: float, lanelet_network: LaneletNetwork
) -> Polygon:
    """
    Get a longitudinal section of a lanelet polygon.

    :param position: Projected center of the lanelet section
    :param lon_length: Total longitudinal length of the lanelet section
    :param lanelet_network: Lanelet network
    :return: Section of the lanelet as a CommonRoad polygon
    """
    position = np.squeeze(position)
    lanelet_ids = lanelet_network.find_lanelet_by_position([position])[0]
    assert len(lanelet_ids) > 0
    lanelet = lanelet_network.find_lanelet_by_id(lanelet_ids[0])
    # Simplify line to avoid errors with duplicate vertices.
    center_line = LineString(lanelet.center_vertices).simplify(0.0)
    s_dist = center_line.project(Point(position))
    start_s = max(0, s_dist - lon_length * 0.5)
    end_s = min(center_line.length, s_dist + lon_length * 0.5)
    start_pt = center_line.interpolate(start_s)
    end_pt = center_line.interpolate(end_s)
    left_line = LineString(lanelet.left_vertices).simplify(0.0)
    left_sub = _sub_line(start_pt, end_pt, left_line)
    right_line = LineString(lanelet.right_vertices).simplify(0.0)
    right_sub = _sub_line(start_pt, end_pt, right_line)
    poly = Polygon(np.array(list(left_sub.coords) + list(right_sub.coords)[::-1]))
    return poly
