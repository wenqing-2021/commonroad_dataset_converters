import os
import re
import warnings
from pathlib import Path
from typing import Optional

import numpy as np
import requests
import typer
from tqdm import tqdm

from commonroad_dataset_converter.mona.mona_utils import (transform_coordinates, _get_map,
                                                          _read_trajectories, _get_mona_config, MONALocation, MONADay,
                                                          MONATrackIdJobGenerator,
                                                          MONADisjunctiveJobGenerator, MONAProcessor, run_processor, )

app = typer.Typer()

file_name_pattern = re.compile(
        r"(?P<location>east|west|merge)_(?P<day>2021080[2-6])_(?P<segment>[0-9]{3})_trajectories")


@app.command()
def convert(location: MONALocation = typer.Argument(..., help="Recording location of the trajectory file"),
        trajectory_file: Path = typer.Argument(..., help="Path to the trajectory file in csv or parquet format"),
        output_dir: Path = typer.Option(".", help="Output directory for scenarios"),
        track_id: Optional[int] = typer.Option(None, help="Track id to use for the planning problem"),
        num_scenarios: Optional[int] = typer.Option(None, help="Number of scenarios to extract"),
        max_duration: Optional[int] = typer.Option(None, help="Maximum duration of a scenario in number of time steps"),
        keep_ego: bool = typer.Option(False, help="Keep the vehicle of the planning problem"),
        disjunctive_scenarios: bool = typer.Option(True, help="Create only non-overlapping scenarios"),
        num_processes: int = typer.Option(1, help="Number of processes to use for conversion"), ):
    """
    Convert MONA recording into CommonRoad scenario(s).

    Nomenclature of created scenarios: [map name][day][segment index]-[map version]_[start frame]_T-[ego id]
    """
    np.random.seed(0)
    output_dir = Path(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    config = _get_mona_config()

    map_scenario = _get_map(config, location.value)

    traj_df = _read_trajectories(trajectory_file)
    transform_coordinates(config["origin"], traj_df, map_scenario)

    match = file_name_pattern.match(trajectory_file.stem)
    if match is not None:
        if match["location"] != location.value:
            warnings.warn(f"Using different map than trajectory location! {match['location']} != {location.value}")
        map_suffix = f'{match["day"]}{match["segment"]}'
    else:
        map_suffix = ""

    processor = MONAProcessor(map_scenario, config, output_dir, keep_ego, map_suffix)

    if disjunctive_scenarios:
        generator = MONADisjunctiveJobGenerator(traj_df, num_scenarios, max_duration)
    else:
        generator = MONATrackIdJobGenerator(traj_df, num_scenarios, track_id)

    run_processor(generator, processor, num_processes)


@app.command()
def download(location: MONALocation = typer.Argument(..., help="Recording location"),
        day: MONADay = typer.Argument(..., help="Recording day"),
        index: int = typer.Argument(..., help="Segment index"),
        output: Path = typer.Argument(..., help="File or directory to save files at"), ):
    """Download trajectory files from server."""
    config = _get_mona_config()
    if output.is_dir():
        output = (output / f"{location.value}_{day.value}_{index:03d}_trajectories.parquet")
    url = (f"https://{config['dataset_url']}/{location.value}/{day.value}/"
           f"{'_'.join([location.value, str(day.value), f'{index:03d}'])}/trajectories.parquet")
    response = requests.get(url, stream=True)
    length = int(response.headers.get("content-length", 0))
    with output.open("wb") as fp:
        with tqdm.wrapattr(fp, "write", total=length, desc="Downloading trajectories", unit_scale=True,
                unit="B", ) as buffer:
            for chunk in response.iter_content(1024):
                buffer.write(chunk)
