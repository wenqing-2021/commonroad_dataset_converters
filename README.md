# Dataset Converters

This repository contains converters from different datasets to CommonRoad scenarios using a common commandline interface.
Currently, we only support the [highD dataset](https://www.highd-dataset.com/)). We will release converters for the [inD dataset](https://www.ind-dataset.com/) and the [INTERACTION dataset](http://interaction-dataset.com/) soon.


### Prerequisites
For the converter you need at least Python 3.6 and the following packages:
* numpy>=1.18.2
* commonroad-io>=2020.2
* pandas>=0.24.2
* scipy>=1.4.1
* ruamel.yaml>=0.16.10

The usage of the Anaconda Python distribution is recommended. 
You can install the required Python packages with the provided requirements.txt file (pip install -r requirements.txt).

### Usage
A conversion can be started from the *dataset_converters* directory by executing  
`python -m src.main dataset input_dir output_dir --num_time_steps_scenario #NUMTIMESTEPSSCENARIO --num_planning_problems #NUMPLANNINGPROBLEMS --keep_ego --obstacle_initial_state_invalid`.

In the following the different parameters are explained:
* **dataset**: The dataset which should be convertered. Currently, only the parameter *highD* for the [highD-dataset](https://www.highd-dataset.com/) is supported. 
This is a mandatory parameter.
* **input_dir**: The directory of the original dataset. This is a mandatory parameter.
* **output_dir**: The directory where the generated CommonRoad scenarios should be stored. This is a mandatory parameter.
* **num_time_steps_scenario**: The maximum length the CommonRoad scenario in time steps . This is an optional parameter. The default length is *150* time steps.
* **num_planning_problems**: The number of planning problems per CommonRoad scenario. This is an optional parameter. The default is *1* planning problem.
* **keep_ego**: Flag to keep vehicles used for planning problems in the scenario. 
This is an optional flag. 
* **obstacle_initial_state_invalid**: Flag to allow for the initial state of an obstacle to start at a nonzero time step. This is an optional flag. 
If not set, the generated CommonRoad scenarios are valid in the sense that all predictions start at time step zero.

A help message is printed by `python -m src.main.py -h`.

If you want to exit/logout from command line, but still want to continue the process execute   
`nohup command-with-options &`.

Note that the specific converters in each subdirectory may host seperate additional scripts and options for conversion.