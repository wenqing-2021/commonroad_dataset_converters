from abc import ABCMeta, abstractmethod
from dataclasses import dataclass
from typing import Generic, Iterable, Tuple, TypeVar

import pandas as pd
from commonroad.scenario.scenario import Scenario


@dataclass
class Window:
    """Contiguous subset of time steps of a dataset, consisting of vehicle states."""

    #: DataFrame with MultiIndex that has the last two levels obstacle id and time step
    #: columns should match CommonRoad state attributes, except x, y which will be converted
    #: into position.
    vehicle_states: pd.DataFrame
    #: DataFrame with Index obstacle id, and at least columns
    #: obstacle_type, width, length
    vehicle_meta: pd.DataFrame
    #: Time step length in seconds.
    dt: float


WindowMetaT = TypeVar("WindowMetaT")


class IWindowGenerator(Generic[WindowMetaT], metaclass=ABCMeta):
    """Subdivide a larger window."""

    @abstractmethod
    def __call__(
        self, window: Window, window_meta: WindowMetaT
    ) -> Iterable[Tuple[Window, WindowMetaT]]:
        pass


class IMetaScenarioCreator(Generic[WindowMetaT], metaclass=ABCMeta):
    """Create the static elements of a dataset window."""

    @abstractmethod
    def __call__(self, window: Window, window_meta: WindowMetaT) -> Scenario:
        pass


class IRecordingGenerator(Generic[WindowMetaT], metaclass=ABCMeta):
    """Iterate over the independent recordings of a dataset."""

    @abstractmethod
    def __iter__(self) -> Iterable[Tuple[Window, WindowMetaT]]:
        """
        Create an iterator that iterates over independent recordings.

        :return: Tuple of the full recordings window and a generic meta information
            object to be used by subsequent steps to identify the recording.
        """
        pass
