import unittest
from pathlib import Path

from commonroad_dataset_converter.datasets.sind.implementation import (
    SindRecordingGenerator,
)
from tests.tools.base_suite import BaseSuite


class TestSindConversion(BaseSuite.ConverterSuite):
    @classmethod
    def setUpClass(cls) -> None:
        cls.dataset = "sind"

    def test_recording_generator(self):
        generator = SindRecordingGenerator(Path(self.input_path), downsample=1)
        for r in generator:
            pass


if __name__ == "__main__":
    unittest.main()
