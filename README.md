# Dataset Converters

This repository contains converters from different datasets to CommonRoad scenarios using a common commandline interface.
Currently, we support the [highD dataset](https://www.highd-dataset.com/)), [inD dataset](https://www.ind-dataset.com/) and the [INTERACTION dataset](http://interaction-dataset.com/). We will release converter for the [Waymo open dataset](https://waymo.com/open/) soon.


### Prerequisites
For the converter you need at least Python 3.6 and the following packages:
* numpy>=1.18.2
* commonroad-io==2020.3
* pandas>=0.24.2
* scipy>=1.4.1
* ruamel.yaml>=0.16.10

The usage of the Anaconda Python distribution is recommended. 
You can install the required Python packages with the provided requirements.txt file (pip install -r requirements.txt).

### Usage
A conversion can be started from the *dataset_converters* directory by executing  
`python -m src.main dataset input_dir output_dir --num_time_steps_scenario #NUMTIMESTEPSSCENARIO --num_planning_problems #NUMPLANNINGPROBLEMS --keep_ego --obstacle_initial_state_invalid`.

In the following the different parameters are explained:
* **dataset**: The dataset which should be convertered. Choice of  *highD*  ,  *inD* , *INTERACTION* .
This is a mandatory parameter.
* **input_dir**: The directory of the original dataset. This is a mandatory parameter.
* **output_dir**: The directory where the generated CommonRoad scenarios should be stored. This is a mandatory parameter.
* **num_time_steps_scenario**: The maximum length the CommonRoad scenario in time steps (int) . This is an optional parameter. The default length is *150* time steps.
* **num_planning_problems**: The number of planning problems per CommonRoad scenario (int). This is an optional parameter. The default is *1* planning problem.
* **keep_ego**: Flag to keep vehicles used for planning problems in the scenario. 
This is an optional flag. The default is *False*.
* **obstacle_start_at_zero**: Flag to require whether the initial state of an obstacle has to start at time step zero. This is an optional flag, default is *False*. 
* **num_processes**: The number of parallel processes for the conversion. (int).This is an optional parameter. The default is *1* process.

Additonal parameters for the highD Dataset only:
* **downsample**: Decrease dt by n * dt. (int) This is an optional flag, default is *1* (no downsampling). 
* **num_vertices**: Number of lane waypoints. (int).This is an optional parameter. The default is *10* lane waypoints.

A help message is printed by `python -m src.main.py -h`.

If you want to exit/logout from command line, but still want to continue the process execute   
`nohup command-with-options &`.

Note that the specific converters in each subdirectory may host seperate additional scripts and options for conversion.