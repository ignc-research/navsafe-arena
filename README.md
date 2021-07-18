
# SafeNav:
SafeNav-arena is a branch task based on [arena-rosnav](https://github.com/ignc-research/arena-rosnav/tree/local_planner_subgoalmode), which is a flexible, high-performance 2D simulator for testing robotic navigation.
The task simulates the crowd navigation using [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros) and tries to train an agent in the crowd navigation to fullfill the hope of safe navigation, that is keeping  safety distance during the robot is navigating to the goal.

Here are some demos about training and evaluation.
 
| <img width="250" height="200" src="https://github.com/ignc-research/navsafe-arena/tree/main/img/raw_random.gif"> | <img width="250" height="200" src="https://github.com/ignc-research/navsafe-arena/tree/main/img/nz_random.gif"> | <img width="250" height="200" src="https://github.com/ignc-research/navsafe-arena/tree/main/img/dz_random.gif"> |
| :----------------------------------------------------------: | :-----------------------------------------------------: | :-----------------------------------------------------: |
| <img width="250" height="200" src="https://github.com/ignc-research/navsafe-arena/tree/main/img/raw.gif"> | <img width="250" height="200" src="https://github.com/ignc-research/navsafe-arena/tree/main/img/nz.gif"> | <img width="250" height="200" src="https://github.com/ignc-research/navsafe-arena/tree/main/img/dz.gif"> |
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

