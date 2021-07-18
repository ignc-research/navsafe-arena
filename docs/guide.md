This documentation will help you set up Arena-Rosnav and provide an overview on how to start a simulation and how to use Arena-Rosnav for training and evaluation of robot navigation. Please also check out the glossary on the bottom of this documentation for better understanding.

# Installation
## Set up Ubuntu in Windows (For Windows users only)
Note: If you already have a Linux distribution, you can continue with **Installation of Arena-Rosnav**.

### 1. Installation of WSL2
Please follow the steps in this [WSL installation guide for Windows 10](https://docs.microsoft.com/en-us/windows/wsl/install-win10) to install WSL2 on your computer.

Note: If you already have a Linux distribution, skip this step.

#### Troubleshooting
You might encounter this problem during installation:
```
Installing, this may take a few minutes...
WslRegisterDistribution failed with error: 0x80370102
Error: 0x80370102 The virtual machine could not be started because a required feature is not installed.
```
This problem can be resolved by enabling CPU virtualization in your BIOS. How you can achieve this depends on your hardware.
[This guide from bleepingcomputer](https://www.bleepingcomputer.com/tutorials/how-to-enable-cpu-virtualization-in-your-computer-bios/) might help you with that.

### 2. Installation of Windows-X-Server
To use WSL with graphical programs, an X-server will need to be installed on the Windows 10 system and the DISPLAY variable will need to be set in Bash/Zsh.
One possible program to use is [VcXsrv](https://sourceforge.net/projects/vcxsrv/).

#### Set up DISPLAY variable
After installing the X-server you need to set the DISPLAY variable in your Bash/Zsh.
Use ```nano ~/.bashrc``` or ```nano ~/.zshrc``` and insert the following code on the bottom of the file.

```export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0```

Exit via Ctrl+X. Agree with Y. Press Enter to save the file name (don't change it).

#### Xlaunch Settings
Start Xlaunch and configure it the following way. In the end the configuration can be saved.
##### Display Settings
- Choose Option: Multiple Windows
- Set Display Number to 0

![image](https://user-images.githubusercontent.com/79201799/117300099-8294d580-ae79-11eb-928a-7d722d150973.png)

##### Client Settings
- Choose Option: Start no Client

![image](https://user-images.githubusercontent.com/79201799/117300149-8e809780-ae79-11eb-8eba-1ff3b16d81b8.png)

##### Extra Settings
- Choose Option: Disable access control

![image](https://user-images.githubusercontent.com/79201799/117300283-b1ab4700-ae79-11eb-8579-00d2d9a829c7.png)

#### Trouble Shooting
If you encounter problems, you might go to Windows Defender Firewall -> Communication between Applications and Windows Firewall.
Look for VcXsrv and change the settings to both private and public checked.

### 3. Installation of Visual Studio Code and WSL-Extension
We recommend you use Visual Studio Code as your programming environment. Please follow the instructions in this [VS Code with WSL tutorial](https://docs.microsoft.com/en-us/windows/wsl/tutorials/wsl-vscode).

### 4. Setup ohmyzsh (optional)
We recommend using ohmyzsh. You can set up ohmyzsh with the following steps.
- Install zsh
```
sudo apt install zsh
```
- Install curl
```
sudo apt install curl
```
- Install ohmyzsh
```
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
```
- Confirm with Y
- Git clone autosuggestions
```
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
```
- Open your .zshrc with editor
```
nano ~/.zshrc
```
- Scroll down to ```plugins```
- Insert ```zsh-autosuggestions``` into ```plugins=()```.
Your plugins might look like this:
```
plugins=(git zsh-autosuggestions)
```
- Exit with Crtl+X
- Confirm with Y
- Confirm with Enter
- Source your .zshrc
```
source $HOME/.zshrc
```

## Installation of Arena-Rosnav
Please install Arena-Rosnav according to the step-by-step instruction in [Installation.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Installation.md).

# Usage
## Common error handling
Different errors can occur during simulation, training or evaluation. Most errors can be resolved by running the following commands in respective order.
- Activate virtual environment
```
workon rosnav
```
- Update git repository
```
cd $HOME/catkin_ws/src/arena-rosnav
git pull
```
- Update ROS workspace
```
cd $HOME/catkin_ws/src/arena-rosnav
rosws update
```
- Build your workspace
```
cd $HOME/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```

Another source of error could be your ```PYTHONPATH```. Please check it with ```echo $PYTHONPATH```. The output should look like this:
```
/home/user/catkin_ws/devel/lib/python3/dist-packages:
/home/user/catkin_ws/src/arena-rosnav:
/home/user/geometry2_ws/devel/lib/python3/dist-packages:
/opt/ros/melodic/lib/python2.7/dist-packages
```
```/opt/ros/melodic/lib/python2.7/dist-packages``` should be listed last. The order of the first 3 paths is not important.

If not, please follow the instructions [Set python path in .zshrc (or .bashrc if you use that)](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Installation.md#13-install-arena-rosnav-repo) in Installation.md.

## Start a simulation
The most basic simulation can be started by using the following command. Please make sure you are working in your virtual environment by running ```workon rosnav``` beforehand.
```
roslaunch arena_bringup start_training.launch  num_envs:=NUM_ENV
```
You can specify the following parameters:
- train_mode:=<true, false>
- useDangerZone:=<true, false>
- useCirclePattern:=<true, false>
- use_viz:=<true, false> (default true)
- local_planner:=<teb,dwa,mpc,cadrl,arena2d> (default dwa)
- task_mode:=<random, manual, scenario> (default random) ~~ (redundant and does not need to be specified anymore)
- map_file:= # e.g. map1 (default map_empty)
- mixRate:= # controls the rate of circle pattern (default 0.3)

## Visualize a simulation
After you start the simulation, you can visulize the simulation by using the following command.
'''
roslaunch arena_bringup visualization_training.launch rviz_file:={NAME_OF_YOUR_RVIZ_FILE} 
'''

The RVIZ will be shown as the following image.
![image](https://user-images.githubusercontent.com/79201799/117291472-58d6b100-ae6f-11eb-8c63-853db1e83fa7.png)

## Training the agent
Please refer to [DRL-Training.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/DRL-Training.md) for detailed explanations about agent, policy and training setups.

During training the agent will at first behave randomly and over time learns to navigate to the goal and (if specified) avoid obstacles. It's not necessary to set any goals. The script will run by itself.

The Quickstart training from [DRL-Training.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/DRL-Training.md) will look like this:

![image](https://user-images.githubusercontent.com/79201799/117296772-be2da080-ae75-11eb-8278-b123093b754d.png)

#### Trouble Shooting
While trying the Quickstart you might encouter the following error in the second terminal:
```
Traceback (most recent call last):
  File "scripts/training/train_agent.py", line 229, in <module>
    treshhold_type="succ", threshold=0.9, verbose=1)
TypeError: __init__() got an unexpected keyword argument 'treshhold_type'
```
This error can be resolved by updating your stable baselines and your workspace. Therefore run the following commands:
```
cd $HOME/catkin_ws/src/forks/stable-baselines3
pip install -e.
```
```
cd $HOME/catkin_ws/src/arena-rosnav
rosws update
```
```
cd $HOME/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Evaluation of agent performance
After training agents, their performance can be evaluated following [Evaluation.md](https://github.com/ignc-research/navsafe-arena/tree/main/docs/Evaluations.md).

![image](https://user-images.githubusercontent.com/79201799/117297905-08635180-ae77-11eb-846c-9b445d4df51a.png)

Note: The evaluation results will be recorded in .rosbag files in the directory in which the rosbag command was run.

At the same time a csv file will be saved in the folder of [deployment](https://github.com/ignc-research/navsafe-arena/tree/main/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scrpits/deployment).

The first one is used as quantitative evaluation, while the second one is used as qualitative evaluation.


## Plotting of the agent performance
The performance of the agent can be visualized by the scripts [plotforcomparison.py](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/arena_navigation/arena_local_planner/evaluation/scripts/semantic_plots/plotforcomparison.py) and [evaluationTrajectory.py](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/eval_human_traj_robot/Evaluation_traj/evaluationTrajectory.py)

