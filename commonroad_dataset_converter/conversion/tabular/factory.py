from abc import abstractmethod
from pathlib import Path
from typing import Callable, Generator, Generic, Iterable, Optional, Tuple, TypeVar

from commonroad.common.file_writer import FileFormat

from commonroad_dataset_converter.conversion.factory import AbstractConverterFactory
from commonroad_dataset_converter.conversion.tabular.planning_problem import (
    EgoPlanningProblemCreator,
    EgoWindow,
    PlanningProblemCreator,
    RandomObstaclePlanningProblemWrapper,
)
from commonroad_dataset_converter.conversion.tabular.prototype_scenario import (
    ScenarioIDCreator,
    ScenarioPrototypeCreator,
)
from commonroad_dataset_converter.conversion.tabular.windowing import (
    DownsamplingWindowWrapper,
    ObstacleWindowGenerator,
    RollingWindowGenerator,
    TimeSliceSamplingWindowGenerator,
    VehicleSamplingWindowGenerator,
)

from .filters import RoutabilityCheck, RoutabilityFilter
from .interface import IMetaScenarioCreator, Window
from .job_consumer import TabularJobConsumer
from .job_producer import TabularJob, TabularJobProducer

WindowMetaT = TypeVar("WindowMetaT")


class TabularConverterFactory(
    AbstractConverterFactory[TabularJob], Generic[WindowMetaT]
):
    """Factory base class for building producer-consumer pipelines in the tabular API."""

    output_dir: Path
    num_time_steps: Optional[int] = 150
    num_planning_problems: int = 1
    downsample: int = 1
    file_writer_type: FileFormat = FileFormat.XML
    max_scenarios: Optional[int] = None
    routability_check: RoutabilityCheck = RoutabilityCheck.Nocheck
    keep_ego: bool = False
    obstacles_start_at_zero: bool = True
    samples_per_recording: Optional[int] = None

    @abstractmethod
    def build_recording_generator(self) -> Iterable[Tuple[Window, WindowMetaT]]:
        pass

    def build_job_generator(self) -> TabularJobProducer:

        recoding_generator = self.build_recording_generator()
        window_generator = self.build_window_generator()

        def generator() -> Generator[Tuple[EgoWindow, WindowMetaT], None, None]:
            for recording in recoding_generator:
                yield from window_generator(*recording)

        return TabularJobProducer(
            self.build_scenario_prototype_creator(),
            self.build_planning_problem_set_creator(),
            generator(),
            self.max_scenarios,
        )

    def build_scenario_prototype_creator(self) -> ScenarioPrototypeCreator:
        return ScenarioPrototypeCreator(
            self.build_scenario_id_creator(),
            self.build_meta_scenario_creator(),
        )

    @abstractmethod
    def build_meta_scenario_creator(self) -> IMetaScenarioCreator:
        pass

    def build_scenario_id_creator(self) -> ScenarioIDCreator:
        return ScenarioIDCreator()

    def build_window_generator(
        self,
    ) -> Callable[[Window, WindowMetaT], Iterable[Tuple[Window, WindowMetaT]]]:
        if self.samples_per_recording is None:
            if self.num_time_steps is None:
                assert self.num_planning_problems == 1
                window_generator = self.build_obstacle_window_generator()
            else:
                window_generator = RollingWindowGenerator(self.num_time_steps)
        else:
            if self.num_time_steps is None:
                assert self.num_planning_problems == 1
                window_generator = VehicleSamplingWindowGenerator(
                    self.samples_per_recording,
                    self.build_obstacle_window_generator(),
                )
            else:
                window_generator = TimeSliceSamplingWindowGenerator(
                    self.num_time_steps, self.samples_per_recording
                )

        if self.downsample > 1:
            window_generator = DownsamplingWindowWrapper(
                window_generator, self.downsample
            )
        return window_generator

    def build_obstacle_window_generator(self):
        return ObstacleWindowGenerator()

    def build_planning_problem_set_creator(self) -> PlanningProblemCreator:
        if self.num_planning_problems <= 0:
            planning_problem_creator = PlanningProblemCreator()
        else:
            planning_problem_creator = EgoPlanningProblemCreator(self.keep_ego)

        if self.num_time_steps is not None:
            planning_problem_creator = RandomObstaclePlanningProblemWrapper(
                planning_problem_creator, self.num_planning_problems
            )

        return planning_problem_creator

    def build_job_processor(self) -> TabularJobConsumer:
        return TabularJobConsumer(
            self.output_dir,
            self.file_writer_type,
            self.obstacles_start_at_zero,
        )

    def build_job_filter(self) -> Callable[[TabularJob], bool]:
        """
        Filter scenarios without planning problems, if window is rolling.

        Explanation: For rolling windows usually the planning problem is not really needed,
        since scenarios are usually just used to represent the dataset in CommonRoad format,
        e.g., for machine learning.

        :return: filter function
        """
        if self.num_time_steps is not None:
            filter_fn = (
                lambda job: len(job.planning_problem_set.planning_problem_dict) > 0
            )
        else:
            filter_fn = super().build_job_filter()

        if self.routability_check != RoutabilityCheck.Nocheck:
            routability_filter = RoutabilityFilter(self.routability_check)

            def wrapper(job: TabularJob) -> bool:
                return filter_fn(job) and routability_filter(job)

            return wrapper
        else:
            return filter_fn
