from unittest import TestCase
from unittest.mock import Mock, patch

import numpy as np
import numpy.testing
import pandas as pd
from commonroad.geometry.shape import Rectangle

from commonroad_dataset_converter.conversion.tabular.planning_problem import (
    EgoPlanningProblemCreator,
    EgoWindow,
    RandomObstaclePlanningProblemWrapper,
)


class TestPlanningProblemCreator(TestCase):
    def setUp(self) -> None:
        self.states = np.empty((10, 6))
        # obstacle id
        self.states[:5, 0] = 0
        self.states[5:, 0] = 1
        # x, y
        self.states[:, 1] = np.arange(10)
        self.states[:, 2] = 0
        # velocity
        self.states[:, 3] = 1.0
        # orientation
        self.states[:, 4] = 0
        # time step
        self.states[:5, 0] = np.arange(5)
        self.states[5:, 0] = np.arange(5)

        vehicle_states = pd.DataFrame(
            self.states,
            columns=["obstacle_id", "x", "y", "velocity", "orientation", "time_step"],
        ).set_index(["obstacle_id", "time_step"])
        vehicle_meta = pd.DataFrame(
            [["car", 2, 5], ["car", 3, 6]],
            columns=["obstacle_type", "width", "length"],
            index=[0, 1],
        )
        self.ego_window = EgoWindow(vehicle_states, vehicle_meta, 1.0, [0])

    def test_ego_planning_problem_creator_keep_ego(self):

        scenario_mock = Mock()
        lanelet_network_mock = Mock()
        lanelet_network_mock.find_lanelet_by_position.return_value = [[None]]
        scenario_mock.lanelet_network = lanelet_network_mock

        creator = EgoPlanningProblemCreator(keep_ego=True)
        with patch(
            "commonroad_dataset_converter.conversion.tabular.planning_problem._cut_lanelet_polygon"
        ) as cut_ll_mock:
            cut_ll_mock.return_value = Rectangle(10, 10, np.array([10, 0]))
            pps = creator(self.ego_window, scenario_mock)
        self.assertEqual(1, len(pps.planning_problem_dict))
        pp = pps.planning_problem_dict[100000]
        numpy.testing.assert_array_equal(pp.initial_state.position, [0, 0])
        self.assertEqual(1, pp.initial_state.velocity)
        self.assertEqual(0, pp.initial_state.orientation)
        self.assertIn(0, self.ego_window.vehicle_meta.index.values)

    def test_ego_planning_problem_creator_no_keep_ego(self):
        scenario_mock = Mock()
        lanelet_network_mock = Mock()
        lanelet_network_mock.find_lanelet_by_position.return_value = [[None]]
        scenario_mock.lanelet_network = lanelet_network_mock

        creator = EgoPlanningProblemCreator(keep_ego=False)
        with patch(
            "commonroad_dataset_converter.conversion.tabular.planning_problem._cut_lanelet_polygon"
        ) as cut_ll_mock:
            cut_ll_mock.return_value = Rectangle(10, 10, np.array([10, 0]))
            pps = creator(self.ego_window, scenario_mock)
        self.assertEqual(1, len(pps.planning_problem_dict))
        pp = pps.planning_problem_dict[100000]
        numpy.testing.assert_array_equal(pp.initial_state.position, [0, 0])
        self.assertEqual(1, pp.initial_state.velocity)
        self.assertEqual(0, pp.initial_state.orientation)
        with self.assertRaises(KeyError):
            self.ego_window.vehicle_meta.loc[0]

    def test_ego_planning_problem_creator_cooperative(self):
        scenario_mock = Mock()
        lanelet_network_mock = Mock()
        lanelet_network_mock.find_lanelet_by_position.return_value = [[None]]
        scenario_mock.lanelet_network = lanelet_network_mock
        self.ego_window.ego = (0, 1)

        creator = EgoPlanningProblemCreator(keep_ego=False)
        with patch(
            "commonroad_dataset_converter.conversion.tabular.planning_problem._cut_lanelet_polygon"
        ) as cut_ll_mock:
            cut_ll_mock.return_value = Rectangle(10, 10, np.array([10, 0]))
            pps = creator(self.ego_window, scenario_mock)
        self.assertEqual(2, len(pps.planning_problem_dict))
        self.assertIn(100000, pps.planning_problem_dict)
        self.assertIn(100001, pps.planning_problem_dict)

    def test_random_planning_problem_wrapper_keep_ego(self):
        scenario_mock = Mock()
        lanelet_network_mock = Mock()
        lanelet_network_mock.find_lanelet_by_position.return_value = [[None]]
        scenario_mock.lanelet_network = lanelet_network_mock

        creator = EgoPlanningProblemCreator(keep_ego=True)
        creator = RandomObstaclePlanningProblemWrapper(creator, num_planning_problems=1)
        with patch(
            "commonroad_dataset_converter.conversion.tabular.planning_problem._cut_lanelet_polygon"
        ) as cut_ll_mock:
            cut_ll_mock.return_value = Rectangle(10, 10, np.array([10, 0]))
            pps = creator(self.ego_window, scenario_mock)
        self.assertEqual(1, len(pps.planning_problem_dict))
        self.assertIn(0, self.ego_window.vehicle_meta.index.values)

    def test_random_planning_problem_wrapper_no_keep_ego(self):
        scenario_mock = Mock()
        lanelet_network_mock = Mock()
        lanelet_network_mock.find_lanelet_by_position.return_value = [[None]]
        scenario_mock.lanelet_network = lanelet_network_mock

        creator = EgoPlanningProblemCreator(keep_ego=False)
        creator = RandomObstaclePlanningProblemWrapper(creator, num_planning_problems=1)
        with patch(
            "commonroad_dataset_converter.conversion.tabular.planning_problem._cut_lanelet_polygon"
        ) as cut_ll_mock:
            cut_ll_mock.return_value = Rectangle(10, 10, np.array([10, 0]))
            pps = creator(self.ego_window, scenario_mock)
        self.assertEqual(1, len(pps.planning_problem_dict))
        self.assertEqual(1, len(self.ego_window.vehicle_meta.index))

    def test_random_planning_problem_wrapper_cooperative_no_keep_ego(self):
        scenario_mock = Mock()
        lanelet_network_mock = Mock()
        lanelet_network_mock.find_lanelet_by_position.return_value = [[None]]
        scenario_mock.lanelet_network = lanelet_network_mock

        creator = EgoPlanningProblemCreator(keep_ego=False)
        creator = RandomObstaclePlanningProblemWrapper(creator, num_planning_problems=2)
        with patch(
            "commonroad_dataset_converter.conversion.tabular.planning_problem._cut_lanelet_polygon"
        ) as cut_ll_mock:
            cut_ll_mock.return_value = Rectangle(10, 10, np.array([10, 0]))
            pps = creator(self.ego_window, scenario_mock)
        self.assertEqual(2, len(pps.planning_problem_dict))
        self.assertEqual(0, len(self.ego_window.vehicle_meta.index))
