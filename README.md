# Dataset Converters

This repository contains converters from different datasets to CommonRoad scenarios using a common commandline interface.
Currently, we support:
- [highD dataset](https://www.highd-dataset.com/),
- [inD dataset](https://www.ind-dataset.com/),
- [INTERACTION dataset](http://interaction-dataset.com/),
- [exiD](https://www.exid-dataset.com/),
- [rounD](https://www.round-dataset.com/),
- [MONA](https://commonroad.in.tum.de/datasets),
- [SinD](https://github.com/SOTIF-AVLab/SinD).

### Install

```bash
pip install commonroad-dataset-converter
```


## Development setup
This project uses [poetry](https://python-poetry.org/). Please follow the instructions to create the virtual development environment.


### Usage
Type following command for help
```bash
crconvert --help
crconvert #dataset --help
```

A conversion can be started from the *dataset_converters* directory by executing  
`crconvert dataset input_dir output_dir --num-time-steps #NUMTIMESTEPSSCENARIO 
--num-planning-problems #NUMPLANNINGPROBLEMS --num-processes #NUMPROCESSES --keep-ego --obstacle-start-at-zero`.

In the following the different parameters are explained:
* **dataset**: The dataset which should be convertered. Currently, parameters *highd*, *ind*, or *INTERACTION* are supported. 
This is a mandatory parameter.
* **input_dir**: The directory of the original dataset. This is a mandatory parameter.
* **output_dir**: The directory where the generated CommonRoad scenarios should be stored. This is a mandatory parameter.
* **num_time_steps_scenario**: The maximum length the CommonRoad scenario in time steps (int) . This is an optional parameter. The default length is *150* time steps.
* **num_planning_problems**: The number of planning problems per CommonRoad scenario (int). This is an optional parameter. The default is *1* planning problem.
* **keep_ego**: Flag to keep vehicles used for planning problems in the scenario. 
This is an optional flag. 
* **obstacle_start_at_zero**: Indicator if the initial state of an obstacle has to start at time step zero. 
This is an optional flag. 
If not set, the generated CommonRoad scenarios will contain predictions start at nonzero time step.
* **num_processes**: The number of parallel processes to run the conversion in order to speed up the conversion. 
This is an optional parameter. The default is *1*
* **inD_all**: (inD) Indicator if convert one CommonRoad scenario for each valid vehicle from inD dataset, 
  since it has less recordings available, note that if enabled, num_time_steps_scenario becomes the minimal number 
  of time steps of one CommonRoad scenario. This is an optional flag. 
* **downsample**: (highD) Downsample the trajectories every N steps, works only for highD converter.
* **downsample**: (highD) Decrease dt by n * dt. (int) This is an optional flag, default is *1* (no downsampling). 
* **num_vertices**: (highD) Number of lane waypoints. (int).This is an optional parameter. The default is *10* lane waypoints.
* **shoulder**: (highD) Adds shoulder lane to map. (bool).This is an optional parameter. The default is *False*.
* **keep_direction**: (highD) Prevents rotating the upper driving direction (right to left) by PI. (bool). This is an optional parameter. The default is *False* so that scenario direction corresponds for all scenarios.
* **routability_check**: (inD) Validity check 'Routability' (int) of the scenario+planning problem from start to goal. 2: strict enforcement of at least one route 0: no checks, default is **2**, strict. requires **commonroad-route-planner** to be installed.

