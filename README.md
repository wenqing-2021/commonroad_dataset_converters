# Dataset Converters

This repository contains tools to convert popular datasets, which are publicly available for scientific purposes only, to the CommonRoad format. Before converting these datasets, please request them on their respective websites. Currently, the [highD](https://www.highd-dataset.com/) and [inD](https://www.ind-dataset.com/) dataset are supported. We will add support for the [INTERACTION](http://interaction-dataset.com/) dataset soon.


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
`python -m src.main dataset input_dir output_dir --num_time_steps_scenario #NUMTIMESTEPSSCENARIO --num_planning_problems #NUMPLANNINGPROBLEMS --num_processes #NUMPROCESSES --keep_ego --obstacle_start_at_zero`.

In the following the different parameters are explained:
* **dataset**: The dataset which should be convertered. Currently, parameters *highD* or *inD* are supported. 
This is a mandatory parameter.
* **input_dir**: The directory of the original dataset. This is a mandatory parameter.
* **output_dir**: The directory where the generated CommonRoad scenarios should be stored. This is a mandatory parameter.
* **num_time_steps_scenario**: The maximum length the CommonRoad scenario in time steps . This is an optional parameter. 
The default length is *150* time steps.
* **num_planning_problems**: The number of planning problems per CommonRoad scenario. This is an optional parameter. 
The default is *1* planning problem.
* **keep_ego**: Flag to keep vehicles used for planning problems in the scenario. 
This is an optional flag. 
* **obstacle_start_at_zero**: Indicator if the initial state of an obstacle has to start at time step zero. 
This is an optional flag. 
If not set, the generated CommonRoad scenarios will contain predictions start at nonzero time step.
* **num_processes**: The number of parallel processes to run the conversion in order to speed up the conversion. 
This is an optional parameter. The default is *1*

A help message is printed by `python -m src.main.py -h`.

If you want to exit/logout from command line, but still want to continue the process execute   
`nohup command-with-options &`.