# Navsafe-Arena:
This repository contains the code for the paper [Enhancing Navigational Safety in Crowded Environments using
Semantic-Deep-Reinforcement-Learning-based Navigation](https://arxiv.org/pdf/2109.11288.pdf).
Navsafe-Arena is based on [arena-rosnav](https://github.com/ignc-research/arena-rosnav/tree/local_planner_subgoalmode), which is a flexible, high-performance 2D simulator for testing robotic navigation, and [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros), which is a pedestrian simulator implementing the social force model. The agent is trained to learn an object-specific navigation behavior, keeping different safety distances towards different types of humans such as adults, children, and elders. An efficient DRL approach called CPU/GPU asynchronous A3C using curriculum learning is utilized. Within the simulation, it is assumed that the agent knows the accurate position and type of humans in the vicinity for understanding the interaction between the robot and humans. In real-world experiments, this information can be acquired using computer vision approaches.


| <img width="250" height="200" src="https://github.com/ignc-research/navsafe-arena/blob/3b7168fe3bdd09674909ddf846894c95a52ccd18/img/raw_random.gif"> | <img width="250" height="200" src="https://github.com/ignc-research/navsafe-arena/blob/3b7168fe3bdd09674909ddf846894c95a52ccd18/img/nz_random.gif"> | <img width="250" height="200" src="https://github.com/ignc-research/navsafe-arena/blob/3b7168fe3bdd09674909ddf846894c95a52ccd18/img/dz_random.gif"> |
| :----------------------------------------------------------: | :-----------------------------------------------------: |:-----------------------------------------------------: |
| <img width="250" height="200" src="https://github.com/ignc-research/navsafe-arena/blob/e39bb9b618e350d521ffc13c0940908c6d583d03/img/raw.gif"> | <img width="250" height="200" src="https://github.com/ignc-research/navsafe-arena/blob/e39bb9b618e350d521ffc13c0940908c6d583d03/img/nz.gif"> | <img width="250" height="200" src="https://github.com/ignc-research/navsafe-arena/blob/e39bb9b618e350d521ffc13c0940908c6d583d03/img/dz.gif"> |
|                            *raw*                             |                      *static zone*                      |                     *dynamic zone*                      |


# Start Guide
We recommend starting with the [start guide](https://github.com/ignc-research/navsafe-arena/tree/main/docs/guide.md) which contains all information you need to know to start off with this project including installation on **Linux and Windows** as well as tutorials to start with. 

* For Mac, please refer to our [Docker](https://github.com/ignc-research/navsafe-arena/tree/main/docs/Docker.md).


## 1. Installation
Please refer to [Installation.md](docs/Installation.md) for detailed explanations about the installation process.

## 1.1. Docker
We provide a Docker file to run our code on other operating systems. Please refer to [Docker.md](docs/Docker.md) for more information.

## 2. Usage

### DRL Training

Please refer to [DRL-Training.md](docs/DRL-Training.md) for detailed explanations about agent, policy and training setups.

**DRL agents** are located in the [agents folder](https://github.com/ignc-research/navsafe-arena/tree/main/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/agents).


##### Quick Start and Supplementary Notes

* In one terminnal, start simulation

```bash
roslaunch arena_bringup start_training.launch num_envs:=1 #switch useDangerZone to be false if normal zone needed
```
* In another terminal, load the pretrained agent

```bash
workon the_name_of_your_virtual_env

roscd arena_local_planner_drl && cd scripts && cd deployment

python run_agent load MLP_HUMAN_DANGER_ZONE -s scenario2 #scenario is not used but should be denoted 
```
* In the third terminal, visualize the simulation

```bash
 roslaunch arena_bringup visualization_training.launch rviz_file:=human_nav
```
