import os
import time
import typer
from enum import Enum
from pathlib import Path
from commonroad_dataset_converter.highD.highd_to_cr import create_highd_scenarios
from commonroad_dataset_converter.inD.ind_to_cr import create_ind_scenarios
from commonroad_dataset_converter.INTERACTION.interaction_to_cr import create_interaction_scenarios

cli = typer.Typer(
    help="Generates CommonRoad scenarios from different datasets"
)


class RoutabilityCheck(str, Enum):
    Nocheck = "nocheck"
    Normal = "normal"
    Strict = "strict"

    def as_int(self) -> int:
        if self == self.Nocheck:
            return 0
        if self == self.Normal:
            return 1
        return 2


@cli.command()
def highD(
        input_dir: Path = typer.Argument(
            ...,
            help="Path to highD-dataset's data folder"
        ),
        output_dir: Path = typer.Argument(
            ...,
            help="Directory to store generated CommonRoad files"
        ),
        num_time_steps: int = typer.Option(
            150,
            help="Maximum number of time steps the CommonRoad scenario can be long"
        ),
        num_planning_problems: int = typer.Option(
            1,
            help="Number of planning problems per CommonRoad scenario"
        ),
        keep_ego: bool = typer.Option(
            False,
            help="Indicator if vehicles used for planning problem should be kept in scenario"
        ),
        obstacle_start_at_zero: bool = typer.Option(
            False,
            help="Indicator if the initial state of an obstacle has to start at time step zero"
        ),
        downsample: int = typer.Option(
            1,
            help="Decrease dt by n*dt"

        ),
        num_processes: int = typer.Option(
            1,
            help="Number of multiple processes to convert dataset"
        ),
        num_vertices: int = typer.Option(
            10,
            help="Number of straight lane waypoints"
        ),
        shoulder: bool = typer.Option(
            False,
            help="Adds shoulder lane to map"
        ),
        keep_direction: bool = typer.Option(
            False,
            help="Prevents rotating the upper driving direction (right to left) by PI"
        ),
        lane_change: bool = typer.Option(
            False,
            help="Whether only use lane changing vehicles as planning problem"
        ),
        extend_width: float = typer.Option(
            0.,
            help="Extend width of the outer lanes [m]"
        )
):
    assert extend_width >= 0.
    os.makedirs(output_dir, exist_ok=True)
    start_time = time.time()
    create_highd_scenarios(
        input_dir,
        output_dir,
        num_time_steps,
        num_planning_problems,
        keep_ego,
        obstacle_start_at_zero,
        num_processes,
        downsample,
        num_vertices,
        shoulder,
        keep_direction,
        lane_change,
        extend_width
    )
    elapsed_time = time.time() - start_time
    print(f"Elapsed time: {elapsed_time} s")


@cli.command()
def inD(
        input_dir: Path = typer.Argument(
            ...,
            help="Path to inD-dataset's data folder"
        ),
        output_dir: Path = typer.Argument(
            ...,
            help="Directory to store generated CommonRoad files"
        ),
        num_time_steps: int = typer.Option(
            150,
            help="Maximum number of time steps the CommonRoad scenario can be long"
        ),
        num_planning_problems: int = typer.Option(
            1,
            help="Number of planning problems per CommonRoad scenario"
        ),
        keep_ego: bool = typer.Option(
            False,
            help="Indicator if vehicles used for planning problem should be kept in scenario"
        ),
        obstacle_start_at_zero: bool = typer.Option(
            False,
            help="Indicator if the initial state of an obstacle has to start at time step zero"
        ),
        num_processes: int = typer.Option(
            1,
            help="Number of multiple processes to convert dataset"
        ),
        all_vehicles: bool = typer.Option(
            False,
            help="Convert one CommonRoad scenario for each valid vehicle from inD dataset, since it has less recordings available. Note that if enabled, num_time_steps_scenario becomes the minimal number of time steps of one CommonRoad scenario"
        ),
        routability_check: RoutabilityCheck = typer.Option(
            RoutabilityCheck.Strict,
            help='Check routability of planning_problem'
        )
):
    os.makedirs(output_dir, exist_ok=True)
    start_time = time.time()
    create_ind_scenarios(
        input_dir,
        output_dir,
        num_time_steps,
        num_planning_problems,
        keep_ego,
        obstacle_start_at_zero,
        num_processes=num_processes,
        inD_all=all_vehicles,
        routability_planning_problem=routability_check.as_int()
    )
    elapsed_time = time.time() - start_time
    print(f"Elapsed time: {elapsed_time} s", end="\r")


@cli.command()
def interaction(
        input_dir: Path = typer.Argument(
            ...,
            help="Path to INTERACTION dataset files"
        ),
        output_dir: Path = typer.Argument(
            ...,
            help="Directory to store generated CommonRoad files"
        ),
        num_time_steps: int = typer.Option(
            150,
            help="Maximum number of time steps the CommonRoad scenario can be long"
        ),
        num_planning_problems: int = typer.Option(
            1,
            help="Number of planning problems per CommonRoad scenario"
        ),
        keep_ego: bool = typer.Option(
            False,
            help="Indicator if vehicles used for planning problem should be kept in scenario"
        ),
        obstacle_start_at_zero: bool = typer.Option(
            False,
            help="Indicator if the initial state of an obstacle has to start at time step zero"
        ),
        num_processes: int = typer.Option(
            1,
            help="Number of multiple processes to convert dataset"
        ),
):
    os.makedirs(output_dir, exist_ok=True)
    start_time = time.time()
    create_interaction_scenarios(
        input_dir,
        output_dir,
        obstacle_start_at_zero=obstacle_start_at_zero,
        num_planning_problems=num_planning_problems,
        keep_ego=keep_ego,
        num_time_steps_scenario=num_time_steps,
        num_processes=num_processes
    )
    elapsed_time = time.time() - start_time
    print(f"Elapsed time: {elapsed_time} s", end="\r")


if __name__ == "__main__":
    cli()
