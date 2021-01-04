# VSS simulation with ROS and Gazebo

[![forthebadge](https://forthebadge.com/images/badges/built-with-science.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/its-not-a-lie-if-you-believe-it.svg)](https://forthebadge.com)

[![ROS version](https://img.shields.io/badge/ROS%20version-noetic-informational?style=for-the-badge)](http://wiki.ros.org/noetic)
[![Gazebo version](https://img.shields.io/badge/Gazebo%20version-11-important?style=for-the-badge)](http://gazebosim.org/)
![Amigos da Poli](./docs/badge_adp.svg)

<!-- ALL-CONTRIBUTORS-BADGE:START - Do not remove or modify this section -->
[![All Contributors](https://img.shields.io/badge/all_contributors-11-orange.svg?style=for-the-badge)](#-contributors)
<!-- ALL-CONTRIBUTORS-BADGE:END -->

IEEE VSS team simulation project with ROS and Gazebo

Para a versão em PT-BR 🇧🇷 desse documento, [veja aqui](./README.pt-br.md)

- [VSS simulation with ROS and Gazebo](#vss-simulation-with-ros-and-gazebo)
  - [📷 Screenshots](#-screenshots)
    - [One robot simulation](#one-robot-simulation)
    - [Team simulation](#team-simulation)
    - [Match simulation](#match-simulation)
  - [🎈 Intro](#-intro)
  - [📣 ROS topics](#-ros-topics)
    - [⬅ Input](#-input)
    - [➡ Output](#-output)
  - [📏 Used models](#-used-models)
    - [© Create your own model](#-create-your-own-model)
  - [🔧 Parameters](#-parameters)
    - [🚀 Roslaunch](#-roslaunch)
  - [📷 Virtual camera](#-virtual-camera)
  - [📁 Folder structure](#-folder-structure)
  - [➕ Dependencies](#-dependencies)
    - [🐍 Python virtual enviroment](#-python-virtual-enviroment)
  - [🎨 Gazebo colors](#-gazebo-colors)
  - [📝 Contributing](#-contributing)
  - [✨ Contributors](#-contributors)

## 📷 Screenshots

### One robot simulation

![screenshot](./docs/screenshot_robot.png)

### Team simulation

![screenshot](./docs/screenshot_team.png)

### Match simulation

![screenshot](./docs/screenshot_match.png)

## 🎈 Intro

It is necessary to clone the project inside a catkin workspace. To create a workspace, refer to [this link](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

To run the simulation with one controlable robot

```bash
roslaunch vss_simulation simulation_robot.launch
```

To run the simulation with the entire team

```bash
roslaunch vss_simulation simulation_team.launch
```

To run the simulation of a match

```bash
roslaunch vss_simulation simulation_match.launch
```

## 📣 ROS topics

### ⬅ Input

The simulation accepts control over **torque** comands (through the interface **effort_controller**) or over **angular velocity** commands (through the interface **velocity_controller**) for the two motors of each robot. Both interfaces are available in the library [ros_control](http://wiki.ros.org/ros_control).

To simulate robots without closed loop rotation simulation, the control over **torque** is more suitable, since torque is approximately proportional to applied tension in a DC motor terminals.

Otherwise, the control interface over **angular velocity** is more suitable.

In both cases, the comands are read from topics of type [std_msgs/Float64](http://docs.ros.org/noetic/api/std_msgs/html/msg/Float64.html)

- **/robot[1..3]/vss_robot_left_controller/command**
- **/robot[1..3]/vss_robot_right_controller/command**
- **/foe[1..3]/vss_robot_left_controller/command**
- **/foe[1..3]/vss_robot_right_controller/command**

### ➡ Output

By default, Gazebo publishes in the topic **/gazebo/model_states** of type [gazebo_msgs/ModelStates](http://docs.ros.org/melodic/api/gazebo_msgs/html/msg/ModelStates.html), with an array of informations about each model in the simulation.

```c
# broadcast all model states in world frame
string[] name                 # model names
geometry_msgs/Pose[] pose     # desired pose in world frame
geometry_msgs/Twist[] twist   # desired twist in world frame
```

For convenience, this package have a script ([vision_proxy.py](./scripts/vision_proxy.py)) that subscribes this topic and republishes the information at diferent topics of type [gazebo_msgs/ModelState](http://docs.ros.org/melodic/api/gazebo_msgs/html/msg/ModelState.html) for each entity (3 robots, 3 foes and 1 ball, 7 in total).

```c
# Set Gazebo Model pose and twist
string model_name           # model to set state (pose and twist)
geometry_msgs/Pose pose     # desired pose in reference frame
geometry_msgs/Twist twist   # desired twist in reference frame
string reference_frame      # set pose/twist relative to the frame of this entity (Body/Model)
                            # leave empty or "world" or "map" defaults to world-frame
```

The republised topics are

- **/vision/robot[1...3]** - Team robots's topics
- **/vision/foe[1...3]** - Adversary robots's topics
- **/vision/ball** - Ball's topic

All units are [SI](https://en.wikipedia.org/wiki/International_System_of_Units), distances are measured in meters, angles in radians, linear velocity in m/s and angular velocity in rad/s.

## 📏 Used models

The simulation is build upon a generic vss robot (more details [here](./urdf/motor.xacro)), inspired by [VSS SDK model](https://github.com/VSS-SDK/VSS-SDK).

As support, models were created for the VSS field and ball, both build from [Robocore's rules](https://www.robocore.net/modules.php?name=Forums&file=download&id=1424) for IEEE VSS.

### © Create your own model

To create a model for your project, refer to:

- [Phobos](https://github.com/dfki-ric/phobos) - Generate urdf files from Blender
- [SW2URDF](http://wiki.ros.org/sw_urdf_exporter) - Generate urdf files from SolidWorks
- [fusion2urdf](https://github.com/syuntoku14/fusion2urdf) - Generate urdf files from Fusion 360
- [ROS wiki](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch) - How build a urdf model from scratch

To use your custom model, change the value of the ```model``` parameter when launching the simulation.

## 🔧 Parameters

### 🚀 Roslaunch

- ```model``` - Path of simulated robot model, default "./urdf/vss_robot.xacro"
- ```debug``` - Enable debug messagens in termianl, default "false"
- ```gui``` - Enable Gazebo's GUI window, default "true"
- ```paused``` - Init simulation paused, default "true"
- ```use_sim_time``` - Use simulated time as reference in messages, default "true"
- ```recording``` - Enable Gazebo's state log, default "false"
- ```keyboard``` - Enable joystick/keyboard control node, default "false"

## 📷 Virtual camera

The simulation have a virtual camera that record images from the top of the field, in the same way as a real VSS match.

The images are published in the topic **/camera/image_raw**

It is possible to watch the footage with the package [image_view](http://wiki.ros.org/image_view)

```sh
rosrun image_view image_view image:=/camera/image_raw
```

## 📁 Folder structure

- **bagfiles/** - Folder to store recorded [bagfiles](./bagfiles/README.md)
- **docs/** - Documentation files
- **launch/** - [Roslaunch](http://wiki.ros.org/roslaunch) files written in ROS [XML syntax](http://wiki.ros.org/roslaunch/XML)
- **meshes/** - .stl files for [vss_generic_robot](./urdf/README.md), created with SolidWorks [SW2URDF](http://wiki.ros.org/sw_urdf_exporter) extension
- **models/** - [Custom Gazebo models](http://gazebosim.org/tutorials?tut=build_model) used inside the simulation, as the field and the VSS ball
- **scripts/** - Python scripts used in the project
  - keyboard_node.py - Pygame script to capture keyboard or joystick input to control the simulation
  - velocity_proxy.py - Script to convert a [std_msgs/Twist](http://docs.ros.org/noetic/api/geometry_msgs/html/msg/Twist.html) message from a [rqt robot steering](http://wiki.ros.org/rqt_robot_steering) to speed or torque comands to 2 motors
  - vision_proxy.py - Script to split [gazebo_msgs/ModelStates](http://docs.ros.org/melodic/api/gazebo_msgs/html/msg/ModelStates.html) array in several [gazebo_msgs/ModelState](http://docs.ros.org/melodic/api/gazebo_msgs/html/msg/ModelState.html) topics

- **urdf/** - Robot description files in [.urdf](http://wiki.ros.org/urdf/XML) and [.xacro](http://wiki.ros.org/xacro) format. The .urdf files were generated with SolidWorks [SW2URDF](http://wiki.ros.org/sw_urdf_exporter) extension
- **worlds/** - World files in [SDL](http://sdformat.org/) format

## ➕ Dependencies

The simulation is develop for ROS and Gazebo, it is recommend to install both with:

```bash
sudo apt install ros-noetic-desktop-full
```

The project depends on the package velocity_controllers and effort_controllers in the library [ros_controllers](https://github.com/ros-controls/ros_controllers) and the python lybrary [pygame](https://github.com/pygame/pygame). It is possible to install both with ```apt-get```

```bash
sudo apt install ros-noetic-velocity-controllers ros-noetic-effort-controllers python3-pygame
```

Or using ```rosdep```

```bash
rosdep install vss_simulation
```

### 🐍 Python virtual enviroment

One may want to run the project inside a python [virtualenv](https://docs.python.org/3/tutorial/venv.html), after all this is a good practice in the python development book

You can create a python enviroment with the command

```sh
python3 -m venv venv
```

Then you should run ```source``` in the virtual enviroment

```sh
source ./venv/bin/activate
```

To install the dependencies, run the command

```sh
pip install -r requirements.txt
```

Some external libraries may be missing to [build](https://stackoverflow.com/questions/7652385/where-can-i-find-and-install-the-dependencies-for-pygame) ```pygame``` package. You can install then with

```sh
sudo apt-get install
  subversion \
  ffmpeg \
  libsdl1.2-dev \
  libsdl-image1.2-dev \
  libsdl-mixer1.2-dev \
  libsdl-ttf2.0-dev \
  libavcodec-dev \
  libavformat-dev \
  libportmidi-dev \
  libsmpeg-dev \
  libswscale-dev \
```

## 🎨 Gazebo colors

For a list of default available color in Gazebo, refert to the config file in the [oficial repo](https://github.com/osrf/gazebo/blob/gazebo11/media/materials/scripts/gazebo.material). We have also 2 OGRE scripts [team blue](./media/materials/scripts/team_blue.material) and [team yellow](./media/materials/scripts/team_yellow.material) for custom colors definition ([Gazebo ref](http://gazebosim.org/tutorials?tut=color_model) and [OGRE ref](http://wiki.ogre3d.org/Materials))

## 📝 Contributing

Any help in the development of robotics is welcome, we encourage you to contribute to the project! To learn how, see the contribution guidelines [here](CONTRIBUTING.md).

## ✨ Contributors

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tr>
    <td align="center"><a href="https://github.com/FelipeGdM"><img src="https://avatars3.githubusercontent.com/u/1054087?v=4?s=100" width="100px;" alt=""/><br /><sub><b>Felipe Gomes de Melo</b></sub></a><br /><a href="https://github.com/thunderatz/vss_simulation/commits?author=FelipeGdM" title="Documentation">📖</a> <a href="https://github.com/thunderatz/vss_simulation/pulls?q=is%3Apr+reviewed-by%3AFelipeGdM" title="Reviewed Pull Requests">👀</a> <a href="https://github.com/thunderatz/vss_simulation/commits?author=FelipeGdM" title="Code">💻</a> <a href="#translation-FelipeGdM" title="Translation">🌍</a></td>
    <td align="center"><a href="https://github.com/LucasHaug"><img src="https://avatars3.githubusercontent.com/u/39196309?v=4?s=100" width="100px;" alt=""/><br /><sub><b>Lucas Haug</b></sub></a><br /><a href="https://github.com/thunderatz/vss_simulation/pulls?q=is%3Apr+reviewed-by%3ALucasHaug" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/Tocoquinho"><img src="https://avatars2.githubusercontent.com/u/37677881?v=4?s=100" width="100px;" alt=""/><br /><sub><b>tocoquinho</b></sub></a><br /><a href="#ideas-Tocoquinho" title="Ideas, Planning, & Feedback">🤔</a> <a href="https://github.com/thunderatz/vss_simulation/commits?author=Tocoquinho" title="Documentation">📖</a></td>
    <td align="center"><a href="https://github.com/Berbardo"><img src="https://avatars0.githubusercontent.com/u/48636340?v=4?s=100" width="100px;" alt=""/><br /><sub><b>Bernardo Coutinho</b></sub></a><br /><a href="https://github.com/thunderatz/vss_simulation/pulls?q=is%3Apr+reviewed-by%3ABerbardo" title="Reviewed Pull Requests">👀</a> <a href="https://github.com/thunderatz/vss_simulation/commits?author=Berbardo" title="Code">💻</a> <a href="https://github.com/thunderatz/vss_simulation/commits?author=Berbardo" title="Documentation">📖</a></td>
    <td align="center"><a href="https://github.com/lucastrschneider"><img src="https://avatars0.githubusercontent.com/u/50970346?v=4?s=100" width="100px;" alt=""/><br /><sub><b>Lucas Schneider</b></sub></a><br /><a href="https://github.com/thunderatz/vss_simulation/pulls?q=is%3Apr+reviewed-by%3Alucastrschneider" title="Reviewed Pull Requests">👀</a> <a href="https://github.com/thunderatz/vss_simulation/commits?author=lucastrschneider" title="Code">💻</a> <a href="#translation-lucastrschneider" title="Translation">🌍</a> <a href="https://github.com/thunderatz/vss_simulation/commits?author=lucastrschneider" title="Documentation">📖</a></td>
    <td align="center"><a href="https://github.com/JuliaMdA"><img src="https://avatars1.githubusercontent.com/u/65100162?v=4?s=100" width="100px;" alt=""/><br /><sub><b>Júlia Mello</b></sub></a><br /><a href="#design-JuliaMdA" title="Design">🎨</a> <a href="#data-JuliaMdA" title="Data">🔣</a></td>
    <td align="center"><a href="https://github.com/ThallesCarneiro"><img src="https://avatars1.githubusercontent.com/u/71659373?v=4?s=100" width="100px;" alt=""/><br /><sub><b>ThallesCarneiro</b></sub></a><br /><a href="#design-ThallesCarneiro" title="Design">🎨</a> <a href="#data-ThallesCarneiro" title="Data">🔣</a></td>
  </tr>
  <tr>
    <td align="center"><a href="https://github.com/TetsuoTakahashi"><img src="https://avatars2.githubusercontent.com/u/38441802?v=4?s=100" width="100px;" alt=""/><br /><sub><b>TetsuoTakahashi</b></sub></a><br /><a href="#ideas-TetsuoTakahashi" title="Ideas, Planning, & Feedback">🤔</a></td>
  </tr>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!
