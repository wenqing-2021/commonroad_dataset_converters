from abc import ABC, ABCMeta, abstractmethod
from typing import Generic, Iterable, TypeVar

JobT = TypeVar("JobT")


class IScenarioJobProducer(ABC, Generic[JobT]):
    """Iterable of independent jobs."""

    @abstractmethod
    def __iter__(self) -> Iterable[JobT]:
        pass


class IScenarioJobConsumer(Generic[JobT], metaclass=ABCMeta):
    @abstractmethod
    def __call__(self, job: JobT) -> None:
        pass
