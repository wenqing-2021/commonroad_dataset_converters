import time
import os
import argparse
from src.highD.highd_to_cr import create_highd_scenarios
from src.inD.ind_to_cr import create_ind_scenarios


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
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
    if args.dataset == "highD":
        create_highd_scenarios(args.input_dir, args.output_dir, args.num_time_steps_scenario,
                               args.num_planning_problems, args.keep_ego, args.obstacle_initial_state_invalid)
    if args.dataset == "inD":
        if not args.obstacle_initial_state_invalid:
            raise NotImplementedError(
                "The inD dataset conversion does not yet allow forcing all obstacles to start at timestep 0"
            )
        create_ind_scenarios(
            args.input_dir,
            args.output_dir,
            [-1],
            -1,
            max_length=args.num_time_steps_scenario,
            multiprocessing=True,
            keep_ego=args.keep_ego
        )
    if args.dataset == "INTERACTION":
        raise NotImplementedError("The interface to the INTERACTION conversion script is not implemented yet")
    else:
        print("Unknown dataset in command line parameter!")

    print("Elapsed time: {} s".format(time.time() - start_time), end="\r")


if __name__ == "__main__":
    main()
