# scalebot

## Overview

The scalebot package was created as a part of the bachelor thesis of Miro Voellmy. It was specifically developed for stair climbing wheelchair Scalevo.

The scalebot package has been tested under [ROS] Indigo and Ubuntu 14.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a [BSD 3-Clause license](ros_package_template/LICENSE).

**Author: Miro Voellmy  
Contact: Miro Voellmy, mvoellmy@ethz.ch  
Affiliation: Autonomous Systems Lab, ETH Zurich**

![Example image](doc/tracks_on_stairs.jpg)


### Publications

If you use this work in an academic context, please cite the following publication(s):
	
* M. Voellmy, P. Fankhauser, M. Hutter, and R. Siegwart: **Stairrecognition on a stair climbing wheelchair using LIDAR Sensors**. IEEE/RSJ International Conference of Placeholders (IROS), 2015. ([PDF](https://www.youtube.com/watch?v=dQw4w9WgXcQ))


## Installation

### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [robot_model](http://github.com/ros/robot_model) (framework for visualization),
- [TCP_Server](https://github.com/Scalevo/TCP_Server) (used for communication to MyRio)
### Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/scalevo/scalaser.git
	cd ../
	catkin_make

### Tests

Package tests have not been implemented yet. You can run this command however to watch Starwars Episode IV in the terminal: 

	telnet towel.blinkenlights.nl


## Usage

To start the live visualization navigate to ~/scalebot/urdf/ and run:

	roslaunch scalebot scalebot.launch model:=scalebot.urdf 

to only use the visualization with the gui run 

	roslaunch scalebot gui_scalebot.launch model:=scalebot.urdf 

## Nodes

### Track_Angle

Main node to publish the angle of the tracks.

#### Subscribed Topics

* **`Lambda`** ([std_msgs/Float64MultiArray])

	Postitions of Lambda motors.

#### Published Topics

* **`joint_states`** ([sensor_msgs/JointState])

	Joint states of the Scalevo Wheelchair. 

#### Services

## Bugs & Feature Requests

Please report bugs and request features using the issue feature of this package.

[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[rviz]: http://wiki.ros.org/rviz
[grid_map_msg/GridMap]: https://github.com/ethz-asl/grid_map/blob/master/grid_map_msg/msg/GridMap.msg
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[visualization_msgs/Marker]: http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
[nav_msgs/OccupancyGrid]: http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
[std_srvs/Empty]: http://docs.ros.org/api/std_srvs/html/srv/Empty.html