# Evaluation Framework for SACPSs
Author: Sebastian Bergemann (sebastian.bergemann@tum.de)

In this repository is actual code of the evaluation framework implemented during the master's thesis of Sebastian Bergemann ("Evaluation Framework for a Self-Adaptive Cyber-Physical System on an Example of a Multi-Robot System"). No further material (final thesis submission, research, etc.) is stored here.

The additional thesis data (organization, analysis, thesis, presentation, etc.) is stored in:

https://gitlab.lrz.de/master-thesis-bergemann/orga-and-material

Readme structure:
1. Overview (Quick Start, Evaluation Framework Description, Structure and Flow)
2. Installation
3. Simulator Adjustments
4. Parameters
5. Requirements to Goal Quality Functions
6. Output
7. Map Layout
8. Important Topics
9. Dirt Generation
10. Map Generation

## Overview

### Quick Start
To use it:
1. Follow the [installation instructions](#sec_installation) below. The framework installation is easy, but the installation of the simulator that should be evaluated is probably more difficult (especially because it needs to be adjusted to follow the standardized IO interface of the framework: see [Simulator Adjustments](#sec_sim_adjust)).
2. Control/Adjust the [parameters.yaml](../launchers/config) (launch path of the simulator, simulation time, map, etc.) - [parameter section](#sec_parameters)
3. Execute the launch script (on the same level as this readme): 
```shell
./main_launch.sh
```
4. After the simulation and evaluation has finished, you can find in the [output folder](../evaluation_output) all important simulation and result data. Most interesting is often the score course over time in [score_evolution folder](../evaluation_output/score_evolution) for a good overview.

If you want to evaluate/simulate one of the two SACPS simulators I used in my thesis (the custom and realistic simulator), you are lucky, because I already modified them to fit to the framework. Therefore, the first step will be easier for you. Furthermore, it is very likely that you find a pre-defined parameters.yaml inside [this folder](../launchers/config/pre-defined_and_saved) with the correct or similar settings you want to use, which will simplify the second step.

However, it is better to have some basic knowledge about the structure of the framework (especially if you are running into some problems), which is briefly described in the following.

![alt text](material/user_walkthrough.png?raw=true)

### Evaluation Framework
Everything is explained (in more detail) in [my thesis](../material/final_thesis.pdf) (especially in chapter 6 regarding the implementation (including interface, installation and simulator modifications)):

https://gitlab.lrz.de/master-thesis-bergemann/orga-and-material/-/blob/master/thesis/final_thesis/Evaluation%20Framework%20for%20a%20SACPS%20on%20an%20Example%20of%20a%20MRS.pdf

From the abstract:

Cyber-Physical Systems (CPS) are present in most areas of our daily life and due to their embedment in the physical world they are exposed to a high level of uncertainty. This causes the need for Self-Adaptive Cyber-Physical Systems (SACPS), which can automatically adapt during run-time to changes of their environment and thus overcome the problems of uncertain and continuously changing environments.

Due to the lack of precise and consistent definitions of adaptivity and what Self-Adaptive Systems (SAS) are exactly, it was discussed how adaptivity can be formally defined and a logical architecture for Self-Adaptive Systems was proposed that should close the gap between the highly abstract definitions and the low-level implementations. During this process, a model problem was defined that is settled in the multi-robot domain. For this model problem, two different simulated Self-Adaptive Cyber-Physical Systems were created. Their differences in setup, launching and simulation prevented a well-defined, useful and comparable evaluation of them in the past. %A framework that can solves this is our major contribution.

***Therefore, we have implemented an evaluation framework that can launch, control and evaluate any Self-Adaptive Cyber-Physical System as long as it adopts the IO interface of the framework, which can easily be integrated for Self-Adaptive Cyber-Physical Systems that are based on the previously mentioned model problem. The framework can perform everything fully autonomously, which includes providing all main inputs for the simulation and recording all required simulation data for the evaluation. The user only needs to adjust the simulation and evaluation settings in one file and gets all needed data visualized for an evaluation analysis. Besides this high usability, the framework provides a high customization while eliminating several limitations of previous projects and still being expandable and scalable for the future. The framework supports the comparison of different Self-Adaptive Cyber-Physical Systems since the evaluation calculation is based on the quality function Q, which can be generally used for every system and use case.***

As secondary contribution, we have improved and modified the two Self-Adaptive Cyber-Physical Systems created for our model problem to enable an evaluation of them with our framework. Some initial evaluation results could already validate previous work and assumptions about Self-Adaptive Systems.


### Workspace Structure
<a name="structure_image"></a>
![alt text](material/structures/directory_overview.png?raw=true)

### Framework Directory Structure
![alt text](material/structures/framework_folder_structure.png?raw=true)

### Launch Flow
![alt text](material/launch_flow/launch_flow_horizontal.png?raw=true)


## Installation 
[My thesis report](../material/final_thesis.pdf) has also an own installation section with 6.2.
<a name="sec_installation"></a>

Since this is a framework and a wrapper for evaluating simulators, the first step is to install the actual simulator. I cannot give any advice for that since the simulators can be very different. If you have luck, I have already worked with the simulator you want to use. In this case I should have made changes inside the simulator and also wrote some notes at the top of the main readme, how to install it (probably just referring to the previous installation guide of the simulator). Some important notes on that: 
- At some point you will be asked to clone the simulator repository. Do it, but do not forget that the cloned repository will be still on the master branch. Since you want to have the changed version with the evaluation, you probably need to switch the git branch after cloning (except it was already merge into the master branch), e.g. 
```shell
git checkout with_eval_framework           #or however this branch is called
```
- If you are instructed to execute `catkin_make`, do it, but it is likely that it will fail since in this changed branch some packages have dependencies on the `wrapper-msgs`, which you will not have until the installation of the evaluation framework in the next step. Just go on (you will execute it later again).

If you have installed it, you can continue with adding the [evaluation framework](https://gitlab.lrz.de/master-thesis-bergemann/evaluation-framework-code) (if you have not cloned it already because it was said in the simulator readme). The whole repository needs to be cloned into the source folder of the simulator workspace like it is just another ROS package. I do not know how the workspace of your simulator looks like, but you can get a general picture of it [here](#structure_image). When you are inside the source folder, you can get it with:
```shell
git clone git@gitlab.lrz.de:master-thesis-bergemann/evaluation-framework-code.git 
```

Make sure all modules and packages are installed (ROS as well as python) and other requirements are met:

Python modules used:

The wrapper scripts are based on python3. So, it should be installed, too. Check it with:
```shell
python3 --version
```
- numpy
- scipy
- matplotlib
- trimesh
- networkx
- pycollada
- opencv-python
- rospkg (should already be installed with ROS)

You can install them with pip3. Get it with:
```shell
sudo apt install python3-pip
```
or if you have it already, check if it is updated:
```shell
pip3 install --upgrade pip
```
and for the modules just:
```shell
pip3 install numpy scipy matplotlib trimesh networkx pycollada opencv-python rospkg
```

ROS packages used (since you should already have installed the simulator, I assume that ROS is correctly installed - preferable ROS melodic):
- rospy (usually pre-installed?)
- rospkg (usually pre-installed?)
- std_msgs (usually pre-installed?)
- geometry_msgs (usually pre-installed?)
- nav_msgs (usually pre-installed?)
- navigation package: https://github.com/ros-planning/navigation.git

As mentioned, most of these ROS packages should actually be installed with basic ROS and the navigation package might already be installed due to the simulator. However, check this and if module/package errors are occurring, get them, e.g. like (or via their repositories):
```shell
sudo apt install ros-melodic-navigation
```

Afterwards you should build the workspace again since new ROS packages and nodes are introduced. In the top of the catkin workspace folder, where the `src` folder is located with its build and devel folders), execute:
```shell
catkin_make 
```

The evaluation framework is set up, but you need to specify first an important path (to the simulator launch file), and you maybe also want to change some general settings/parameters.
You can do this in one file: `parameters.yaml` (inside `launchers/config` or this [link](../launchers/config)). If you want to use a pre-defined one, you need to replace the file mentioned above with it. More about parameters in the [parameter section](#sec_parameters) below.

If you are using a simulator that I already have adjusted (you will know this), you can directly move on to launching everything. Otherwise the simulator needs to be adjusted so that it takes the provided inputs from the wrapper (evaluation framework) and also publish everything the framework needs to evaluate it. For this purpose, follow first the next section. 

Finally, you can start the framework with the simulator by executing either the shell script (which also source the setup.bash):
```shell
./main_launch.sh
```
or by launching directly the main launch file (but then you might need to source the /devel/setup.bash of your simulator and export TURTLEBOT3_MODEL=burger separately):
```shell
roslaunch launchers main_launcher.launch 
```

## Simulator Adjustments <a name="sec_sim_adjust"></a>
The best way to adjust a simulator/SACPS to the evaluation framework is to check out chapter 6.4. of my thesis. There, I also explain it with actual code snippets.

Maybe you need to create a ROS launch file, which then calls the actual launch script of the simulator (see `example_for_sim.launch`), because the wrapper can only call a ROS launch file. If you have already a ROS launch file that can start the whole simulation, it is perfectly fine, but you need the integrate some parameters in it (see the main launch file adjustments below).

The simulator does not need to get and transform the map from the map_server or the initial image and it also does not need to generate the dirt/tasks itself anymore. These two inputs will be given now from the wrapper. Also the values from the global parameter server (of the wrapper) should be used instead of own parameters (if they overlap).

Furthermore, the simulator needs to publish some data in a specific form. See the section with important topics below.

Only when these requirements are satisfied, the evaluation framework can work/evaluate properly.


### Adjustments in the main launch file:
The main launch file with which the simulator can be launched, needs to provide these parameters, which will be then overridden by the wrapper (otherwise the launch will fail):
```xml
<arg name="sim_gui" default="true"/> 
<arg name="sim_seed" default="100"/> 
<arg name="sim_no_of_robots" default="2"/> 
<arg name="sim_r0_x_start" default="-4.0"/> 
<arg name="sim_r0_y_start" default="4.5"/> 
<arg name="sim_r1_x_start" default="3.0"/> 
<arg name="sim_r1_y_start" default="2.5"/> 
<arg name="sim_base_map_file" default="$(find robot_meta)/maps/reference_map.yaml"/> 
<arg name="sim_gazebo_world_file" default="$(find launch_simulation)/world/small_environment.world"/> 
```

The default values can be set as you like. Furthermore, these parameters should be used by the simulator, especially the map and world file as well as the start positions. So, give the sim_base_map_file to your map_server (if you have/need one) and the sim_gazebo_world_file to your gazebo simulator (if you have/launch one).

## Parameters <a name="sec_parameters"></a>
All needed/possible parameters are set and can be changed in the `parameters.yaml` file inside the [launchers pkg](../launchers/config) (`launchers/config/parameters.yaml`).

The usage is explained in the file. After launching the evaluation, the ROS parameter server is loaded with this parameter file and from then on, all wrapper nodes will take these parameters. Also the simulator can access these parameters globally. However, they should not be changed or deleted.

Although everything is well described inside the parameter file itself, I just want to stress that you definitely need to specify the location of the simulator launch file. Otherwise the wrapper cannot launch the simulator! You can do this in two ways:
- Add the absolute path to the ROS launch file of the simulator behind `absolute_path_to_simulator_launch`. Example:
```yaml
absolute_path_to_simulator_launch: "/home/user/catkin_ws/src/launch-pkg/launch-path/simulation.launch"
```
- or describe it relatively with the ROS package of it (`pkg_of_simulator_launch`) and then the internal path to it inside this package (`internal_path_to_simulator_launch`). Of course, this only works if the launch file is located inside a referable ROS package. Example:
```yaml
pkg_of_simulator_launch: "launch-pkg"
internal_path_to_simulator_launch: "launch-path/simulation.launch"
```

## Requirements to Goal Quality Functions
The evaluation is based on the quality function Q (see papers of Ana Petrovska and background sections 3.2. and 3.3. of my thesis). The natural requirements need to be translated to goal quality functions to enable the evaluation with the Q score. The setup of the goal quality functions is use case specific and I will only refer to my own final selection. This "translation" is explained in detail in section 5.3.3. of my thesis, but I will shortly list it up here, too:

Functional requirements to business goals:
- The system should work as expected. This means on a functional level that no major errors and robot crashes happen.

    Mapped quality function inside parameters.yaml: ***no_crashes***

- The system should work as expected regarding the tasks. This means that a minimum of dirt objects needs to be cleaned to consider the system as working.

    Mapped quality function inside parameters.yaml: ***minimum_cleaned***

Non-functional requirements to adaptation goals:
- The system should clean as much dirt as possible.

    Mapped quality function inside parameters.yaml: ***higher_cleaning_rate***

- The system should clean dirt as fast as possible.

    Mapped quality function inside parameters.yaml: ***indirectly included in shorter_dirt_existence***

- The system should explore as much space as possible to not miss dirt.

    Mapped quality function inside parameters.yaml: ***indirectly included in shorter_dirt_existence***

- The system should detect as much dirt as possible.

    Mapped quality function inside parameters.yaml: ***higher_detection***

- The system should not ignore dirt (at least not for a long time).

    Mapped quality function inside parameters.yaml: ***shorter_dirt_existence***

- The system should cost as less money as possible during run-time. In our case, we reduce this to travel as less as possible, because the more the robots travel (unnecessarily), the more energy they consume and energy costs money.

    Mapped quality function inside parameters.yaml: ***less_distance***



## Output <a name="sec_output"></a>
In the [output folder](../evaluation_output) you can find after a simulation/evaluation run everything important (you need the run ID, which you can find in the beginning and at the end of a simulation run in the terminal):
- the [parameters](../evaluation_output/config_info) with which the run was executed
- the used [transformed map](../evaluation_output/transformed_maps)
- the [dirt distribution pattern](../evaluation_output/dirt_distributions)
- the [recorded data](../evaluation_output/recorded_data) (mandatory + optional)
- the [simulation history](../evaluation_output/simulation_history) with the most important locations over time (as mp4 video)
- the [score results](../evaluation_output/results)
- the score results in [summary](../evaluation_output/result_summaries) with the parameters and recordings
- the score results as [diagrams](../evaluation_output/score_evolution) (scores courses over time)


## Map layout and other information about it (also written in map_provider)
If you are familiar with the map layout provided by the official ROS map_server: this is exactly the same layout!

If "map" is the received object (OccupancyGrid taken from the PROVIDED_MAP_TOPIC):\
- The start point of the map (first element of the array) is the bottom left corner of the initial image of the map (identical with the layout of the map from map_server).
- This bottom left corner (first array element) maps to the real world position given by map.info.origin.
- From this origin / start point the x direction is horizontal (to the right) and the y direction is vertical (to the top).
- Each cell has the size map.info.resolution x map.info.resolution (resolution is in m/cell).
- The map is then a rectangle (or normally square) which has from the start point <map.info.width> cells in x direction and <map.info.height> in y direction.
- If you want to have the real world distances in m: length/width = map.info.width * map.info.resolution and height = map.info.height * map.info.resolution

![alt text](material/map_flow/map_layout_overview.png?raw=true)

If you have a real world position (x,y) and want to get the cell containing this position:
```python
cell_x = min(int((x - map.info.origin.position.x) / map.info.resolution), map.info.width-1)
cell_y = min(int((y - map.info.origin.position.y) / map.info.resolution), map.info.height-1)
index = cell_x + cell_y * map.info.width
map.data[index]
```

If you have a cell index of the map/grid array and want to know the real world position (x, y) in m:
(The position will be the bottom left corner of the cell. To get the whole area of the cell, expand the position by map.info.resolution in x and in y direction)
```python
cell_x = int(index % map.info.width)    #[number of cells in x direction]
cell_y = int(index / map.info.width)    #[number of cells in y direction]
x = map.info.origin.position.x + cell_x * map.info.resolution
y = map.info.origin.position.y + cell_y * map.info.resolution
```

For path planning and dirt generation I recommend using the center of the cells:
The resulting center of cell map.data[index] is in real world:
```python
cell_x = int(index % map.info.width)    #[number of cells in x direction]
cell_y = int(index / map.info.width)    #[number of cells in y direction]
x = map.info.origin.position.x + (cell_x + 0.5) * map.info.resolution
y = map.info.origin.position.y + (cell_y + 0.5) * map.info.resolution
```

## Important topics for the simulator:
Attention: all of the topics have the wrapper namespace as prefix. You have to add it always since the simulator is in another namespace.
The wrapper namespace is by default `/wrapper/` and can be change in the first launch file (main_launcher.launch in ./launchers/launch/). How to get parameters (especially for the topics):
```python
# Get wrapper namespace from the parameter server:
wrapper_namespace = rospy.get_param("/wrapper_namespace")
# Get any other parameter from the parameter server (now always with the wrapper namespace in front og it!):
rospy.get_param("/" + wrapper_namespace + "/parameter_name")
# Get a complete topic of the interface between wrapper and simulator:
provided_map_topic = "/" + wrapper_namespace + "/" + rospy.get_param("/" + wrapper_namespace + "/provided_map_topic")
```

The following topic names are only the final ones of my work. THey might be changed in the meantime. Better checkout the parameters.yaml, because in there all topic names are stored (which is why you should always request the topics from there and not hard code them!).

You should (or might) listen to (this is described in more detail in chapter 6 of my thesis):
- New dirt will be spawned to `new_dirt` (of type DirtObject from wrapper_msgs.msg). As soon as it is published, it should be spawned/handled by the simulator. [parameter: `new_dirt_topic`]
- The (transformed/scaled) map can be received from the `provided_map` as OccupancyGrid (from nav_msgs.msg). It has all the important meta data like origin, resolution and width/height, as well as the actual occupancy status of each cell (in an array [check out the map layout and code samples in this readme]). [parameter: `provided_map_topic`]
- If you need directly the initial image of the map, you can either take it manually from the structure or get it from `provided_map_path` (of type String() from std_msgs.msg with absolute path). [parameter: `provided_map_path_topic`]
- Simulator shutdown signal is a message of type Empty() from std_msgs.msg on topic `simulator_shutdown`. The wrapper will terminate the provided launch file of the simulator. This should normally be enough to shutdown all simulator nodes. However, if your simulator has some nodes which cannot be killed by the launch file (blocked resources, etc.), you can modify them so that they listen to the topic above and then start an internal shutdown. [parameter: `shutdown_sim_topic`]
- Final shutdown signal is a message of type Empty() from std_msgs.msg on topic `global_shutdown`. However, this should only be needed for the wrapper. The simulator should have terminated before that! [parameter: `final_shutdown_topic`]

You should publish:
- Indicate the time when your simulator has finished its setup and the robots are able to move. This can be done by sending a message of type Empty() from std_msgs.msg on topic `sim_setup_finished`. [parameter: `sim_setup_end_topic`]
- Currently active dirt (undiscovered or discovered, but not completed) should be published continuously to `active_dirt` (of type DirtObjectList from wrapper_msgs.msg). This is needed to prevent duplicates (multiple dirt piles at the same position/cell) while generating new dirt. [parameter: `active_dirt_topic`]

## Dirt Generation
![alt text](material/dirt_flow/dirt_generation_flow.png?raw=true)

## Map Generation
![alt text](material/map_flow/map_generation_flow.png?raw=true)