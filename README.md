# vio_initialization_evaluation

This repository includes
* Specific dataset for VIO initialization evaluation
* VIO initialization evaluation metrics
* The source code for "Benchmark for Evaluating Initialization of Visual-Inertial Odometry"

1. [Install](#install)
2. [Prepare the Data](#prepare-the-data)
   * [Poses](#poses)
   * [Evaluation Parameters](#evaluation-parameters)
   * [Start and end times](#start-and-end-times)
3. [Run the Evaluation](#run-the-evaluation)
   * [Single Trajectory Estimate](#single-trajectory-estimate)
   * [Multiple Trajectory Estimate](#multiple-trajectory-estimates)
4. [Utilities](#utilities)
   * [Dataset Tools](#dataset-tools)
   * [Misc. Scripts](#misc-scripts)
5. [Customization: `Trajectory` class](#customization)
6. [Credits](#credits)

## 1. Prerequisites
The package is written in python and tested in Ubuntu 18.04.

### Python
Currently only `python2` is supported.
* `numpy` and `matplotlib` for the analysis/plotting
* `colorama` for colored console output

### ROS
The package can be used as a ROS package as well as a standalone tool.
To use it as a ROS package, simply clone it into your workspace.

## 2. Data Orgnization
Each folder needs to contain two text files specifying the groundtruth and initialization estimated poses with timestamps.

* `groundtruth.txt`: groundtruth poses
* `init_estimate.txt`: initialization estimated poses

For analyzing results from `N` runs, the estimated poses should have suffixes `0` to `N-1`.
You can see the folders under `results` for examples.
These files contains **all the essential information** to reproduce quantitative trajectory evaluation results with the toolbox.

### Poses
The groundtruth (`groundtruth.txt`) and initialization estimated poses (`init_estimate.txt`) are specified in the following format

```
# timestamp tx ty tz qx qy qz qw
1403636859.536666 4.621150000 -1.837605000 0.739627000 -0.129040000 -0.810903000 -0.062030000 0.567395000
......
```

Note that the file is space separated, and the quaternion has the `w` component at the end.
The timestamps are in the unit of second and used to establish temporal correspondences.

## 3. Running the Initialization Evaluation
As a ROS package, run

```
rosrun rpg_trajectory_evaluation initialization_evaluation.py <result_folder>
```

or as a standalone package, run

```
python2 scripts/initialization_evaluation.py <result_folder> 
```

`<result_folder>` should contain the groundtruth, initialization estimate as mentioned above.

### Output
After the initialization evaluation is done, you will find sub-scale plot folders under `<result_folder>`

### Parameters
* `--pdf`: save plots as pdf instead of png. Default: `False`

## 4. Dataset tools
Under `scripts/dataset_tools`, we provide several scripts to prepare your dataset for analysis. Specifically:
* `kitti_to_euroc`: convert KITTI style format to the EuRoC format.
* `rosbag_to_euroc.py`: extract data in a ROS bag to the EuRoC format.
* `timestamps_visualization.py`: visualizae the timestamps in a data file to check rrame skipping and frame dropping consditions before evaluation.