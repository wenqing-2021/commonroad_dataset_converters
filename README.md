# Dataset Converters

This repository contains converters from different datasets to CommonRoad scenarios using a common commandline interface.
Currently, we support:

-   [highD](https://www.highd-dataset.com/),
-   [inD](https://www.ind-dataset.com/),
-   [INTERACTION](http://interaction-dataset.com/),
-   [exiD](https://www.exid-dataset.com/),
-   [rounD](https://www.round-dataset.com/),
-   [MONA](https://commonroad.in.tum.de/datasets),
-   [SIND](https://github.com/SOTIF-AVLab/SinD).

## Install

```bash
pip install commonroad-dataset-converter
```

### Install with support for nuplan dataset

```bash
pip install commonroad-dataset-converter[nuplan]
```

## Development setup

This project uses [poetry](https://python-poetry.org/). Please follow the instructions to create the virtual development environment.

## Usage

Type following command for help

```bash
$ crconvert --help
Usage: crconvert [OPTIONS] INPUT_DIR OUTPUT_DIR COMMAND [ARGS]...

  Generates CommonRoad scenarios from different datasets

Arguments:
  INPUT_DIR   Path to the data folder  [required]
  OUTPUT_DIR  Directory to store generated CommonRoad files  [required]

Options:
  --num-time-steps INTEGER        Maximum number of time steps in the
                                  CommonRoad scenario  [default: 150]
  --num-planning-problems INTEGER
                                  Number of planning problems per CommonRoad
                                  scenario. More than one creates a
                                  cooperative scenario.  [default: 1]
  --keep-ego / --no-keep-ego      Vehicles used for planning problem should be
                                  kept in the scenario.  [default: no-keep-
                                  ego]
  --obstacles-start-at-zero / --no-obstacles-start-at-zero
                                  The lowest time step in the scenario is set
                                  to zero.  [default: no-obstacles-start-at-
                                  zero]
  --downsample INTEGER            Decrease dt by n*dt  [default: 1]
  --num-processes INTEGER         Number of processes to convert dataset.
                                  [default: 1]
  --all-vehicles / --no-all-vehicles
                                  Create one planning problem/scenario for
                                  each valid vehicle. Invalidates num-time-
                                  steps.  [default: no-all-vehicles]
  --routability-check [nocheck|strict]
                                  Check routability of planning_problem
                                  [default: RoutabilityCheck.Strict]
  --output-type [xml|pb]          File type of CommonRoad scenarios  [default:
                                  xml]
  --max-scenarios INTEGER         Only create up to n scenarios.
  --samples-per-recording INTEGER
                                  Randomly sample n scenarios from each
                                  recording.
  --help                          Show this message and exit.

Commands:
  exid         Convert the exiD dataset into CommonRoad scenario(s).
  highd        Convert the highD dataset into CommonRoad scenario(s).
  ind          Convert the inD dataset into CommonRoad scenario(s).
  interaction  Convert the INTERACTION dataset into CommonRoad scenario(s).
  mona         Convert the MONA dataset into CommonRoad scenario(s).
  round        Convert the rounD dataset into CommonRoad scenario(s).
  sind         Convert the SIND into CommonRoad scenario(s).

```

## Docker

The dataset converter can also be used within a docker container. We provide a [Dockerfile](./Dockerfile) to build the image.

```bash
docker build -t commonroad-dataset-converter .
```

The image's entrypoint calls `crconvert`. Any arguments passed to `docker run` are passed to the executable. To access the original dataset and converted scenarios, provide it as a volume mount `-v PATH_TO_DATASET:/data:ro` and `-v PATH_TO_OUTPUT_DIRECTORY:/output`

Example: Convert the highD dataset

```bash
docker run -v `pwd`/highd_dataset:/data:ro -v `pwd`/highd_scenarios:/output commonroad-dataset-converter \
    --max-scenarios 20 --samples-per-recording 20 /data /output highd
```
