import unittest
from pathlib import Path

from commonroad_dataset_converter.datasets.mona.implementation import (
    MonaRecordingGenerator,
)
from tests.tools.base_suite import BaseSuite


class TestMona(BaseSuite.ConverterSuite):
    @classmethod
    def setUpClass(cls) -> None:
        cls.dataset = "mona"

    def test_recording_generator(self):
        generator = MonaRecordingGenerator(Path(self.input_path))
        for r in generator:
            pass


if __name__ == "__main__":
    unittest.main()
