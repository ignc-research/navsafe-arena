## Evaluations
We provide tools to evaluate the planners.
### Usage
#### 1) Start Simulation
```
roslaunch arena_bringup start_training.launch num_env:=1
```
Explanation:
* num_env:=1: only 1 environment is required during evaluation
#### 2) Record Rosbags
```
rosbag record -o Name_of_your_bag /scenario_reset -e "(.*)police(.*)"
```
Explanation:
* this command will record all topics necessary for evaluation
* -e "(.*)police(.*)": records all topics containing "police"
* Name_of_your_bag: name of bag file

