import os
import glob
import time
import argparse
import multiprocessing
import numpy as np

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.scenario.scenario import Scenario, Tag

from commonroad_dataset_converter.helper import load_yaml


def get_args() -> argparse.Namespace:
    """
    Specifies and reads command line arguments

    :return: command line arguments
    """
    parser = argparse.ArgumentParser(description="Recalculate obstacle positions according to velocities",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('input_dir', type=str, help='Path to dataset files')
    parser.add_argument('output_dir', type=str, help='Directory to store generated CommonRoad files')
    parser.add_argument('--num_processes', type=int, default=1,
                        help='Number of multiple processes to convert dataset')

    return parser


def fix_obstacle(scenario: Scenario):
    for obstacle in scenario.dynamic_obstacles:
        old_state = obstacle.initial_state
        for state in obstacle.prediction.trajectory.state_list:
            x = old_state.position[0] + old_state.velocity * np.cos(old_state.orientation) * scenario.dt
            y = old_state.position[1] + old_state.velocity * np.sin(old_state.orientation) * scenario.dt
            state.position = np.array([x, y])
            old_state = state

    return scenario


def fix_scenario(filename, highd_config, output_dir):
    scenario, planning_problem_set = CommonRoadFileReader(filename).open()
    scenario = fix_obstacle(scenario)
    tags = {Tag(tag) for tag in highd_config.get("tags")}
    fw = CommonRoadFileWriter(scenario, planning_problem_set, highd_config.get("author"),
                              highd_config.get("affiliation"), highd_config.get("source"), tags)
    filename = os.path.join(output_dir, "{}.xml".format(scenario.scenario_id))
    fw.write_to_file(filename, OverwriteExistingFile.ALWAYS, check_validity=False)
    print("Scenario file stored in {}".format(filename))


def main(args):
    start_time = time.time()

    # make output dir
    os.makedirs(args.output_dir, exist_ok=True)

    #
    fns = sorted(glob.glob(os.path.join(args.input_dir, "*.xml")))
    highd_config = load_yaml(os.path.dirname(os.path.abspath(__file__)) + "/config.yaml")

    if args.num_processes < 2:
        for fn in fns:
            fix_scenario(fn, highd_config, args.output_dir)
    else:
        with multiprocessing.Pool(processes=args.num_processes) as pool:
            pool.starmap(
                fix_scenario,
                [
                    (
                        fn,
                        highd_config,
                        args.output_dir
                    )
                    for fn in fns
                ]
            )

    print("Elapsed time: {} s".format(time.time() - start_time), end="\r")


if __name__ == "__main__":
    # get arguments
    args = get_args().parse_args()
    main(args)
