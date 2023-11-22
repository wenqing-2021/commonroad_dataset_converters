from dataclasses import dataclass
from multiprocessing import Pool
from typing import (
    Callable,
    Collection,
    Generic,
    Iterable,
    Sequence,
    Sized,
    TypeVar,
    Union,
)

from tqdm import tqdm

JobT = TypeVar("JobT")


@dataclass
class WorkerRunner(Generic[JobT]):
    """Run a single producer-multi consumer scheme.

    If only one consumer process is requested, the scheme does not use the multiprocessing library.
    This is useful for debugging.
    """

    generator: Union[Iterable[JobT], Collection[JobT]]
    processor: Callable[[JobT], None]
    #: Number of consumer processes.
    num_processes: int = 1

    def run(self) -> None:
        num_processes = self.num_processes
        if hasattr(self.generator, "__len__"):
            num_scenarios = len(self.generator)
            num_processes = min(self.num_processes, num_scenarios)
        else:
            num_scenarios = None

        if num_processes > 1:
            with Pool(num_processes) as pool:
                iterator = pool.imap_unordered(self.processor, self.generator)
                for _ in tqdm(
                    iterator,
                    total=num_scenarios,
                    desc="Creating scenarios",
                    unit="scenarios",
                ):
                    pass
        else:
            for params in tqdm(
                self.generator,
                total=num_scenarios,
                desc="Creating scenarios",
                unit="scenarios",
            ):
                self.processor(params)
