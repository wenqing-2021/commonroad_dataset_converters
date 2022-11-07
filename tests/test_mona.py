import unittest
import tempfile
from pathlib import Path

from commonroad_dataset_converter.main import sind, mona
from commonroad_dataset_converter.mona.mona_utils import MONALocation


class TestMona(unittest.TestCase):

    def setUp(self) -> None:
        path_prefix = Path(__file__).parent / "resources" / "mona"
        self.east_dataset = path_prefix / "east_20210802_000_trajectories.parquet"
        self.west_dataset = path_prefix / "west_20210802_000_trajectories.parquet"
        self.merge_dataset = path_prefix / "merge_20210802_000_trajectories.parquet"

    def _test(self, location, dataset):
        with tempfile.TemporaryDirectory() as output_dir:
            output_path = Path(output_dir)
            mona(location, dataset, output_path, track_id=None, max_duration=None, num_scenarios=10, num_processes=1,
                 keep_ego=False, disjunctive_scenarios=True)
            self.assertEqual(len(list(output_path.glob("*.xml"))), 10)

    def test_west(self):
        self._test(MONALocation.west, self.west_dataset)

    def test_east(self):
        self._test(MONALocation.east, self.east_dataset)

    def test_merge(self):
        self._test(MONALocation.merge, self.merge_dataset)

    def test_multi_process(self):
        with tempfile.TemporaryDirectory() as output_dir:
            output_path = Path(output_dir)
            mona(MONALocation.west, self.west_dataset, output_path, track_id=None, max_duration=None, num_scenarios=10, num_processes=4,
                 keep_ego=False, disjunctive_scenarios=True)
            self.assertEqual(len(list(output_path.glob("*.xml"))), 10)


if __name__ == '__main__':
    unittest.main()
