![Capa](../docs/banner_urdf.png)

# 📏 Generic VSS Robot

## 📂 Overview

Inside this folder are the description files for a Generic VSS Robot

- **generic\_vss\_robot.urdf** - [URDF](http://wiki.ros.org/urdf) description of the robot, generated with [SW2URDF](http://wiki.ros.org/sw_urdf_exporter) extension for SolidWorks
- **generic\_vss\_robot.xacro** - Main file that imports all other files
- **generic\_vss\_robot.gazebo** - Gazebo-specific configurations i.e. physical properties and colors configuration
  - **generic\_vss\_team.gazebo** - Different color configurations for both teams (Yellow/Blue) and each robot (0..2) inside the team.
- **motor.xacro** - Configuration of actuator and transmission element for motor control with [hardware\_interface](http://wiki.ros.org/ros_control#Hardware_Interfaces) lib from [ros\_control](http://wiki.ros.org/ros_control)
- **generic_vss_robot.pdf** - Detailed blueprint from the 3D model

Inside the **meshes/** folder, are the .stl files for each part of the robot and inside **media/materials/scripts/** are [OGRE scripts](http://wiki.ogre3d.org/Materials) to define different colors and textures.

## 📜 Main parameters

|         Parameter          |          Value | Unit  |
| :------------------------: | -------------: | :---- |
|        Wheel radius        |             25 | mm    |
|      Wheel thickness       |              8 | mm    |
|     Wheels separation      |             55 | mm    |
|       Wheels density       |           1150 | kg/m³ |
|     Wheels mass (each)     |             18 | g     |
|      Wheels material       |          Nylon | \-    |
|       Body material        | 50% infill ABS | \-    |
|        Body density        |            510 | kg/m³ |
|        Total height        |             62 | mm    |
|        Total width         |             78 | mm    |
|        Total length        |             78 | mm    |
|         Total mass         |            180 | g     |
| Total inertia momentum Izz |          0.113 | g m²  |

## 🟣 Motor parameters

The model's motor is inspired in [Pololu's 50:1 Micro Metal Gearmotor](https://www.pololu.com/product/3073) in order to achieve realistic values.

|           Parameter            | Value | Unit   |
| :----------------------------: | ----: | :----- |
|        Motor max torque        |    73 | mN m   |
| Robot max linear acceleration  |    16 | m/s²   |
| Robot max angular acceleration |  1420 | rad/s² |
|        Motor max speed         |   650 | RPM    |
|     Robot max linear speed     |   1.7 | m/s    |
|    Robot max angular speed     |   9.8 | rad/s  |

The values were estimated from the relations derived neglecting the wheel's momento of inertia in y axis (motor's rotation axis)

<img src="https://render.githubusercontent.com/render/math?math=\ddot{x} m=T/R_{wheel}">
<img src="https://render.githubusercontent.com/render/math?math=\dot{\omega} I_{zz}=T S_{wheels}/2 R_{wheel}">

Where "S wheels" is the separation between both wheels
