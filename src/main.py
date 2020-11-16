import os
import time
import argparse

from src.highD.highd_to_cr import create_highd_scenarios
from src.inD.ind_to_cr import create_ind_scenarios
from src.INTERACTION.converter_INTERACTION import convert


def get_args() -> argparse.Namespace:
    """
    Specifies and reads command line arguments

    :return: command line arguments
    """
    parser = argparse.ArgumentParser(description="Generates CommonRoad scenarios different datasets")
    parser.add_argument('dataset', type=str, choices=["inD", "highD", "INTERACTION"], help='Specification of dataset')
    parser.add_argument('input_dir', type=str, help='Path to dataset files')
    parser.add_argument('output_dir', type=str, help='Directory to store generated CommonRoad files')
    parser.add_argument('--num_time_steps_scenario', type=int, default=150,
                        help='Maximum number of time steps the CommonRoad scenario can be long')
    parser.add_argument('--num_planning_problems', type=int, default=1,
                        help='Number of planning problems per CommonRoad scenario')
    parser.add_argument('--keep_ego', default=False, action='store_true',
                        help='Indicator if vehicles used for planning problem should be kept in scenario')
    parser.add_argument('--obstacle_initial_state_invalid', default=False, action='store_true',
                        help='Indicator if the initial state of an obstacle has to start at time step zero')

    return parser.parse_args()


def main():
    start_time = time.time()

    # get arguments
    args = get_args()

    # make output dir
    os.makedirs(args.output_dir, exist_ok=True)

    if args.dataset == "highD":
        create_highd_scenarios(args.input_dir, args.output_dir, args.num_time_steps_scenario,
                               args.num_planning_problems, args.keep_ego, args.obstacle_initial_state_invalid)
    elif args.dataset == "inD":
        create_ind_scenarios(
            args.input_dir,
            args.output_dir,
            num_time_steps_scenario=args.num_time_steps_scenario,
            num_planning_problems=args.num_planning_problems,
            keep_ego=args.keep_ego,
            obstacle_initial_state_invalid=args.obstacle_initial_state_invalid
        )
    elif args.dataset == "INTERACTION":
        convert(args.input_dir, args.output_dir)
    else:
        print("Unknown dataset in command line parameter!")

    print("Elapsed time: {} s".format(time.time() - start_time), end="\r")


if __name__ == "__main__":
    main()
