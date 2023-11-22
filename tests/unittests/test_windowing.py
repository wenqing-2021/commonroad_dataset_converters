from unittest import TestCase
from unittest.mock import MagicMock, Mock

import numpy.testing
import pandas as pd

from commonroad_dataset_converter.conversion.tabular.interface import Window
from commonroad_dataset_converter.conversion.tabular.windowing import (
    DownsamplingWindowWrapper,
    ObstacleWindowGenerator,
    RollingWindowGenerator,
    TimeSliceSamplingWindowGenerator,
    VehicleSamplingWindowGenerator,
)


class TestWindowGenerator(TestCase):
    def setUp(self) -> None:
        self.t = [0, 1, 2, 3, 4, 5, 6]
        vehicle_states = pd.DataFrame(
            self.t,
            columns=["t"],
            index=pd.MultiIndex.from_tuples(
                [(0, 0), (0, 1), (0, 2), (0, 3), (1, 4), (1, 5), (1, 6)]
            ),
        )
        vehicle_meta = pd.DataFrame(
            ["car", "car"], columns=["obstacle_type"], index=[0, 1]
        )
        self.recording = Window(vehicle_states, vehicle_meta, 1.0)

    def test_obstacle_call(self):
        window_generator = ObstacleWindowGenerator()
        windows = list(window_generator(self.recording, None))
        self.assertEqual(2, len(windows))
        self.assertEqual(4, len(windows[0][0].vehicle_states.index))
        numpy.testing.assert_array_equal(
            windows[0][0].vehicle_states.t.values, self.t[:4]
        )
        self.assertEqual(3, len(windows[1][0].vehicle_states.index))
        numpy.testing.assert_array_equal(
            windows[1][0].vehicle_states.t.values, self.t[4:]
        )

    def test_rolling_call(self):
        window_generator = RollingWindowGenerator(window_length=5)
        windows = list(window_generator(self.recording, None))
        self.assertEqual(2, len(windows))
        self.assertEqual(5, len(windows[0][0].vehicle_states.index))
        numpy.testing.assert_array_equal(
            windows[0][0].vehicle_states.t.values, self.t[:5]
        )
        self.assertEqual(2, len(windows[1][0].vehicle_states.index))
        numpy.testing.assert_array_equal(
            windows[1][0].vehicle_states.t.values, self.t[5:]
        )

    def test_downsampling_call(self):
        mock_window_generator = MagicMock()
        mock_window_generator.return_value = [(self.recording, None)]
        window_generator = DownsamplingWindowWrapper(mock_window_generator, 2)
        windows = list(window_generator(self.recording, None))
        self.assertEqual(1, len(windows))
        self.assertEqual(4, len(windows[0][0].vehicle_states.index))
        numpy.testing.assert_array_equal(
            windows[0][0].vehicle_states.t.values, self.t[::2]
        )

    def test_vehicle_sampling_call(self):
        window_generator = VehicleSamplingWindowGenerator(
            samples_per_recording=1, base_window_generator=ObstacleWindowGenerator()
        )
        windows = list(window_generator(self.recording, None))
        self.assertEqual(1, len(windows))

    def test_time_slice_sampling_call(self):
        window_generator = TimeSliceSamplingWindowGenerator(
            window_length=2, samples_per_recording=2
        )
        windows = list(window_generator(self.recording, None))
        self.assertEqual(2, len(windows))
        self.assertEqual(2, len(windows[0][0].vehicle_states.index))
        self.assertEqual(2, len(windows[1][0].vehicle_states.index))
