# 🛄 ROS bagfiles

The generated bagfiles should be stored in this folder

## Instructions

A helpfull tool in ROS development is the creation of bagfiles with help of [rosbag](http://wiki.ros.org/rosbag)

To generate a bagfile from the active simulation, run the command

```sh
rosbag record -a
```

All topics being published will be recorded in the bagfile. It is possible to select specific topics to be saved with the syntax

```sh
rosbag record -O subset /topic1 /topic2
```

To get more information about a rosbag, use the command

```sh
rosbag info <filename>
```

More details in how to use rosbags [here](http://wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data)
