# Unscented Kalman Filter for Tracking Moving Objects

---

In this project, Unscented Kalman filter is used to estimate the state of a moving object of interest(e.g, pedestrian, vehicles, or other moving 
objects ) with noisy LiDAR and RADAR measurements. For the motion model, CTRV(Constant Turn Rate and Velocity) model is used.


## Contents 
- `src` - directory with the project code
- `data`  - directory with input files containing noisy lidar and radar measurements
- `output` -  directory with output and log files
- `Docs` - directory with input-output files formats description

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4
* Eigen library

## Getting started

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. There are 
   some sample inputs in `data/` folder. Also, specify output filename in `output/` folder.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt ../output/state_estimate.csv`
   
   
