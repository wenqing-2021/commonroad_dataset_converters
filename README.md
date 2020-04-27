# Dataset Converters

This repository contains converters from different datasets to CommonRoad scenarios using a common commandline interface.  
For the converter you need at least Python 3.6 and the following packages:
* numpy>=1.18.2
* commonroad-io>=2020.2
* pandas>=0.24.2
* scipy>=1.4.1
* ruamel.yaml>=0.16.10

The usage of the Anaconda Python distribution is recommended. 
You can install the required Python packages with the provided requirements.txt file (pip install -r requirements.txt).

A conversion can be started from the *dataset_converters* directory by executing  
`python -m src.main dataset input_dir output_dir --num_time_steps_scenario #NUMTIMESTEPSSCENARIO --num_planning_problems #NUMPLANNINGPROBLEMS --keep_ego --obstacle_initial_state_invalid`.

In the following the different parameters are explained:
* **dataset**: The dataset which should be convertered. Currently, only the parameter *highD* for the [highD-dataset](https://www.highd-dataset.com/) is supported. 
This is a mandatory parameter.
* **input_dir**: The directory of the original dataset. This is a mandatory parameter.
* **output_dir**: The directory where she generated CommonRoad scenarios should be stored. This is a mandatory parameter.
* **num_time_steps_scenario**: The maximum number of time steps the CommonRoad scenario can be long. This is an optional parameter. The default length is *150* time steps.
* **num_planning_problems**: The number of planning problems per CommonRoad scenario. This is an optional parameter. The default is *1* planning problem.
* **keep_ego**: Indicator if vehicles used for planning problem should be kept in scenario. 
This is an optional parameter. This boolean parameter needs no additional value. 
If the parameter is added the parameter evaluates to true otherwise to false. Therefore, the default is *false*.
* **obstacle_initial_state_invalid**: Indicator if the initial state of an obstacle has to start at time step zero. This is an optional parameter. 
If the parameter is added the parameter evaluates to true otherwise to false. 
Therefore, the default is *false* which results in a valid CommonRoad scenarios in which all predictions start at time step zero.

You can list the different parameters by executing `python main.py -h`.

If you want to exit/logout from command line, but still want to continue the process execute   
`nohup command-with-options &`.