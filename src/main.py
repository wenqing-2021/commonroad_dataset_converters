import os
import time
import argparse
import warnings

from src.highD.highd_to_cr import create_highd_scenarios
from src.inD.ind_to_cr import create_ind_scenarios
from src.INTERACTION.interaction_to_cr import create_interaction_scenarios


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
                        help='Maximum number of time steps the CommonRoad scenario can be long, default=150')
    parser.add_argument('--num_planning_problems', type=int, default=1,
                        help='Number of planning problems per CommonRoad scenario, default=1')
    parser.add_argument('--keep_ego', default=False, action='store_true',
                        help='Indicator if vehicles used for planning problem should be kept in scenario, '
                             'default=False')
    parser.add_argument('--obstacle_start_at_zero', default=False, action='store_true',
                        help='Indicator if the initial state of an obstacle has to start at time step zero, '
                             'default=False')
    parser.add_argument('--num_processes', type=int, default=1,
                        help='Number of multiple processes to convert dataset, '
                             'default=1')
    parser.add_argument('--downsample', type=int, default=1, help='Decrease dt by n*dt')

    return parser.parse_args()


def main():
    start_time = time.time()

    # get arguments
    args = get_args()

    # make output dir
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
    if args.dataset == "highD":
        create_highd_scenarios(args.input_dir, args.output_dir, args.num_time_steps_scenario,
                               args.num_planning_problems, args.keep_ego, args.obstacle_start_at_zero,
                               args.num_processes, args.downsample)
    elif args.dataset == "inD":
        if args.downsample != 1:
            warnings.warn('Downsampling only implemented for highD. Using original temporal resolution!')
        create_ind_scenarios(args.input_dir, args.output_dir, args.num_time_steps_scenario,
                             args.num_planning_problems, args.keep_ego, args.obstacle_start_at_zero,
                             num_processes=args.num_processes)
    elif args.dataset == "INTERACTION":
        create_interaction_scenarios(args.input_dir, args.output_dir,
                                     obstacle_start_at_zero=args.obstacle_start_at_zero,
                                     num_planning_problems=args.num_planning_problems, keep_ego=args.keep_ego,
                                     num_time_steps_scenario=args.num_time_steps_scenario, num_processes=args.num_processes)
    else:
        print("Unknown dataset in command line parameter!")

    print("Elapsed time: {} s".format(time.time() - start_time), end="\r")


if __name__ == "__main__":
    main()
