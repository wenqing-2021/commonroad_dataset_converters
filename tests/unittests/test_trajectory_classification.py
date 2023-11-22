from unittest import TestCase

import numpy as np
import numpy.testing

from commonroad_dataset_converter.conversion.util.trajectory_classification import (
    _calc_curvature,
)


class TestTrajectoryClassification(TestCase):
    def test__calc_curvature(self):
        radius = 10.0
        angle = np.linspace(0, 0.5 * np.pi, 10)
        left_turn = np.column_stack(
            (radius * np.cos(angle), radius * np.sin(angle), np.arange(10))
        )
        right_turn = left_turn.copy()
        right_turn[:, 1] *= -1
        straight = np.column_stack((np.arange(10), np.arange(10), np.arange(10)))

        curvature = _calc_curvature(left_turn)
        numpy.testing.assert_allclose(1 / radius, curvature[2:-2])

        curvature = _calc_curvature(right_turn)
        numpy.testing.assert_allclose(-1 / radius, curvature[2:-2])

        curvature = _calc_curvature(straight)
        numpy.testing.assert_allclose(0.0, curvature[2:-2])
