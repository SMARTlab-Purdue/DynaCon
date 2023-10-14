# **DynaCon: Dynamic Robot Planner with Contextual Awareness via LLMs**

**Authors:** [Gyeongmin Kim](https://github.com/gmkim97), [Taehyeon Kim](https://github.com/QualiaT), Shyam Sundar Kannan, Vishnunandan L. N. Venkatesh, Donghan Kim and Byung-Cheol Min.

Submitted to IEEE International Conference on Robotics and Automation (ICRA), 2024

[Project Page](https://sites.google.com/view/dynacon) | [arXiv](https://arxiv.org/abs/2309.16031) | [Video](https://www.youtube.com/watch?v=Yo8SWcJYCLU)

**Abstract:** Mobile robots often rely on pre-existing maps for effective path planning and navigation. However, when these maps are unavailable, particularly in unfamiliar environments, a different approach become essential. This paper introduces DynaCon, a novel system designed to provide mobile robots with contextual awareness and dynamic adaptability during navigation, eliminating the reliance of traditional maps. DynaCon integrates real-time feedback with an object server, prompt engineering, and navigation modules. By harnessing the capabilities of Large Language Models (LLMs), DynaCon not only understands patterns within given numeric series but also excels at categorizing objects into matched spaces. This facilitates dynamic path planner imbued with contextual awareness. We validated the effectiveness of DynaCon through an experiment where a robot successfully navigated to its goal using reasoning.

**This package is for ROS Noetic.**

## Setup
### Install dependencies
```
$ pip install -r requirments.txt
```

### Install husky_ur3_simulator package
[husky_ur3_simulator](https://github.com/QualiaT/husky_ur3_simulator)

### Install aws-robomaker-small-house-world
[aws-robomaker-small-house-world](https://github.com/aws-robotics/aws-robomaker-small-house-world)

### Other settings
- Setting pattern-based reasoning simulation world
```
$ cd worlds_for_pattern_reasoning/world/
$ mv * ~/catkin_ws/src/husky_ur3_simulator/husky_ur3_gazebo/worlds
$ cd ..
$ cd launch/
$ mv * ~/catkin_ws/src/husky_ur3_simulator/husky_ur3_gazebo/launch
$ cd ..
$ cd ..
```

- Setting categorical reasoning simulation world
```
$ cd worlds_for_categorical_reasoning/world/
$ mv * ~/catkin_ws/src/husky_ur3_simulator/husky_ur3_gazebo/worlds
$ cd ..
$ cd launch/
$ mv * ~/catkin_ws/src/husky_ur3_simulator/husky_ur3_gazebo/launch
$ cd ..
$ cd ..
```

- Install obj_server ROS package
```
$ mv obj_server ~/catkin_ws/src
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```

## Creating OpenAI API Key
The code relies on OpenAI API. Create an API Key at https://platform.openai.com/.

Create a file named ```api_key.txt``` in the root folder of the project and paste your OpenAI Key in the file. 

## How to start?

```
- Bring up Gazebo with the robot model
$ roslaunch husky_ur3_gazebo husky_ur3_test_world_01.launch

- Bring up RViz
$ roslaunch husky_ur3_gripper_moveit_config Omni_control.launch

- Bring up move_base
$ roslaunch husky_ur3_nav_without_map execution_without_map.launch

- Run navigation script
$ python3 scripts/navigation_with_gpt.py

- Run LLM
$ python3 scripts/run_llm_with_ROS.py --expt-name test

- Bring up obj_server
$ roslaunch obj_server broad_test01_1.launch
$ roslaunch obj_server server.launch
```

