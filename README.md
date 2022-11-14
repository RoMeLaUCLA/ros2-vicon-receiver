# Vicon receiver for ROS2 ![GitHub tag (latest by date)](https://img.shields.io/github/v/tag/andreacamisa/ros2-vicon-receiver)

**ros2-vicon-receiver** is a ROS2 package, written in C++, that retrieves data from Vicon software and publishes it on ROS2 topics. The code is partly derived from a mixture of [Vicon-ROS2](https://github.com/aheuillet/Vicon-ROS2) and [Vicon bridge](https://github.com/ethz-asl/vicon_bridge) .

This is NOT an official ROS2 package and is not supported. The package has been successfully tested with ROS2 Dashing Diademata, ROS2 Foxy and ROS2 Galactic on the operating systems Ubuntu 18.04 Bionic Beaver, Ubuntu 20.04 Focal Fossa and MacOS 10.13 High Sierra.

## Installation of ROS2

**If ROS2 is already installed, you can skip this section**

## Ubuntu 18.04 Bionic Beaver
If you are using Ubuntu 18.04 Bionic Beaver, you can install all the dependencies by simply `cd`'ing into the main project folder and then running
```
$ ./install_ubuntu_bionic.sh
```

## Ubuntu 20.04 Foxy Focal
If you are using Ubuntu 20.04 Foxy Focal, you can install all the dependencies by simply `cd`'ing into the main project folder and then running
```
$ ./install_ros2_foxy.sh
```

### Installation of Datastream SDK and other libraries

The Datastream SDK libraries are required to be installed in the system. You can find them on [the official website](https://www.vicon.com/software/datastream-sdk/?section=downloads).

This package is shipped with Datastream SDK 10.1 (the latest version at the time of writing). If you are running Linux x64 and you want to install this version, simply `cd` into the main project folder and issue the command
```
$ ./install_libs.sh
```

## Quick start

### Building the package

:warning: Do not forget to source the ROS2 workspace: `source /opt/ros/dashing/setup.bash` or `source /opt/ros/foxy/setup.bash`

Build the executable
```
$ colcon build --symlink-install
```

### Running the program

Open a new terminal and source the project workspace:
```
$ source install/setup.bash
```

To run the program, use the [launch file template](vicon_receiver/launch/client.launch.py) provided in the package. First, open the file and
edit the parameters. The hostname of the motion capture computer in RoMeLa is `192.168.200.101` while it is connected to the Robocup network. *If this
setup changes, you must change this setting.* Running `colcon build` is not needed because of the `--symlink-install` option previously used.

Now you can launch the program with
```
$ ros2 launch vicon_receiver client.launch.py
```

### Checking the topics
In another terminal, source the ROS2 workspace with 
```
$ source install/setup.bash
```

If the Vicon motion capture software is running on the RoMeLa computer, then you can see the topics using:
```
$ ros2 topic list
```


Exit the program with CTRL+C.

### Information on ROS2 topics and messages

The **ros2-vicon-receiver** package creates a topic for each segment in each subject with the pattern `namespace/subject_name/segment_name`.
Information is published on the topics as soon as new data is available from the vicon client (typically at the vicon client frequency). Two
topics will be published for each subject: the centroidal position of the object based on the markers under `namespace/subject_name/subject_name` with message type
[Position](vicon_receiver/msg/Position.msg) and a list of marker positions uner `namespace/subject_name/markers` with message type
[MarkerPositions](vicon_receiver/msg/MarkerPositions.msg) and 

For the current implementation in RoMeLA, the default namespace `vicon` will be used. If you have two subjects (`subject_1` and `subject_2`,
then, the topic published will be as follows:
```
vicon/subject_1/subject_1
vicon/subject_1/markers
vicon/subject_2/subject_2
vicon/subject_2/markers
```

## Contributors
**ros2-vicon-receiver** is developed by
[Andrea Camisa](https://www.unibo.it/sitoweb/a.camisa),
[Andrea Testa](https://www.unibo.it/sitoweb/a.testa) and
[Giuseppe Notarstefano](https://www.unibo.it/sitoweb/giuseppe.notarstefano)

## Acknowledgements
This result is part of a project that has received funding from the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement No 638992 - OPT4SMART).

<p style="text-align:center">
  <img src="logo_ERC.png" width="200" />
  <img src="logo_OPT4Smart.png" width="200" /> 
</p>
