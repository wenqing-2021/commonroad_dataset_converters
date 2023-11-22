import logging
from dataclasses import dataclass
from typing import Generic, Iterable, Optional, Tuple, TypeVar

import pandas as pd
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario

from ..interface import IScenarioJobProducer
from .planning_problem import EgoWindow, PlanningProblemCreator
from .prototype_scenario import ScenarioPrototypeCreator

_logger = logging.getLogger(__name__)


WindowMetaT = TypeVar("WindowMetaT")


@dataclass
class TabularJob:
    """Scenario job of the tabular API."""

    #: DataFrame with MultiIndex that has the last two levels obstacle id and time step
    #: columns should match CommonRoad state attributes, except x, y which will be converted
    #: into position.
    vehicle_states: pd.DataFrame
    #: DataFrame with Index obstacle id, and at least columns
    #: obstacle_type, width, length
    vehicle_meta: pd.DataFrame
    #: Empty base scenario containing all static objects.
    meta_scenario: Scenario
    #: Planning problem set written to the scenario file.
    planning_problem_set: PlanningProblemSet


@dataclass
class TabularJobProducer(IScenarioJobProducer[TabularJob], Generic[WindowMetaT]):
    meta_scenario_generator: ScenarioPrototypeCreator[WindowMetaT]
    planning_problem_generator: PlanningProblemCreator
    window_generator: Iterable[Tuple[EgoWindow, WindowMetaT]]
    #: The maximum number of jobs to process. None means unlimited.
    max_jobs: Optional[int]

    def __iter__(self) -> Iterable[TabularJob]:
        for i, (window_job, window_meta) in enumerate(self.window_generator):
            meta_scenario = self.meta_scenario_generator(window_job, window_meta)
            planning_problem = self.planning_problem_generator(
                window_job, meta_scenario
            )
            job = TabularJob(
                window_job.vehicle_states,
                window_job.vehicle_meta,
                meta_scenario,
                planning_problem,
            )
            yield job
            if self.max_jobs is not None and i + 1 >= self.max_jobs:
                break
