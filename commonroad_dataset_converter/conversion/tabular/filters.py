from dataclasses import dataclass
from enum import Enum

from commonroad_route_planner.route_planner import RoutePlanner

from .job_producer import TabularJob


class RoutabilityCheck(str, Enum):
    Nocheck = "nocheck"
    # Normal = "normal"
    Strict = "strict"


@dataclass
class RoutabilityFilter:
    routability_type: RoutabilityCheck

    def __call__(self, job: TabularJob) -> bool:
        """
        Checks if a planning problem is routable on scenario

        :return: bool, True if CommonRoad planning problem is routeable with max_difficulty
        """
        if self.routability_type == RoutabilityCheck.Nocheck:
            return True

        # TODO: Distinction between regular and strict?
        for planning_problem in job.planning_problem_set.planning_problem_dict.values():
            route_planner = RoutePlanner(
                job.meta_scenario,
                planning_problem,
                backend=RoutePlanner.Backend.NETWORKX_REVERSED,
            )
            candidate_holder = route_planner.plan_routes()
            _, num_candidates = candidate_holder.retrieve_all_routes()

            if num_candidates <= 0:
                return False

        return True
