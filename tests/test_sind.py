import unittest
import tempfile
from pathlib import Path

from commonroad_dataset_converter.main import sind


class TestSind(unittest.TestCase):

    def setUp(self) -> None:
        self.test_dataset = Path(__file__).parent / "resources" / "sind"

    def test_single_process(self):
        with tempfile.TemporaryDirectory() as output_dir:
            output_path = Path(output_dir)
            sind(self.test_dataset, output_path, track_id=None, max_duration=None, num_scenarios=10, num_processes=1,
                 keep_ego=False, disjunctive_scenarios=True)
            self.assertEqual(len(list(output_path.glob("*.xml"))), 10)

    def test_multi_process(self):
        with tempfile.TemporaryDirectory() as output_dir:
            output_path = Path(output_dir)
            sind(self.test_dataset, output_path, track_id=None, max_duration=None, num_scenarios=10, num_processes=4,
                 keep_ego=False, disjunctive_scenarios=True)
            self.assertEqual(len(list(output_path.glob("*.xml"))), 10)


if __name__ == '__main__':
    unittest.main()
